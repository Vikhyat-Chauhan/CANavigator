# hydra_teleop/logging/async_logger.py
import logging
import logging.handlers
import queue
import threading
import time
import atexit
from pathlib import Path
from typing import Optional
import json
from datetime import datetime

# ---- Config dataclass (simple) ----
class AsyncLoggerCfg:
    def __init__(
        self,
        logfile: str = "hydra.log",
        max_bytes: int = 10 * 1024 * 1024,
        backup_count: int = 5,
        queue_maxsize: int = 2000,
        drop_on_full: bool = True,
        console: bool = True,
        level: int = logging.INFO,
        monitor_interval_s: Optional[float] = 5.0,  # None disables monitor
        register_atexit: bool = True,               # auto-shutdown on process exit
        ensure_dirs: bool = True,                   # create parent dirs for logfile
        json_format: bool = True,                   # <-- now respected
    ):
        self.logfile = logfile
        self.max_bytes = max_bytes
        self.backup_count = backup_count
        self.queue_maxsize = queue_maxsize
        self.drop_on_full = drop_on_full
        self.console = console
        self.level = level
        self.monitor_interval_s = monitor_interval_s
        self.register_atexit = register_atexit
        self.ensure_dirs = ensure_dirs
        self.json_format = json_format


class JsonFormatter(logging.Formatter):
    """Structured JSON logs with extras preserved (minus noisy internals)."""

    _SKIP_KEYS = {
        # std logging internals
        "args","msg","message","asctime","exc_info","exc_text",
        "stack_info","stacklevel","msecs","relativeCreated",
        "pathname","filename","module","lineno","funcName",
        "created","thread","threadName","process","processName",
        # also skip these by request / redundancy
        "levelname","levelno","taskName","logger"
    }

    def format(self, record: logging.LogRecord) -> str:
        # --- choose what goes into the 'msg' field ---
        if hasattr(record, "message_json"):
            # explicit override via extra={"message_json": {...}}
            msg_payload = record.message_json
        else:
            raw = getattr(record, "msg", None)
            if isinstance(raw, (dict, list)):
                # if you pass a dict/list as the log message, keep it structured
                msg_payload = raw
            else:
                # fallback: the usual formatted string
                msg_payload = record.getMessage()

        payload = {
            "ts": datetime.fromtimestamp(record.created).isoformat(timespec="milliseconds"),
            "level": record.levelname,
            "msg": msg_payload,
        }

        # Sweep remaining attributes to capture structured extras
        for k, v in record.__dict__.items():
            if k in self._SKIP_KEYS:
                continue
            if v is None:
                continue
            try:
                json.dumps({k: v})  # test serializability
                payload[k] = v
            except Exception:
                payload[k] = str(v)

        if record.exc_info:
            payload["exc_info"] = self.formatException(record.exc_info)

        return json.dumps(payload, ensure_ascii=False)


# ---- Custom non-blocking QueueHandler with drop-on-full policy ----
class NonBlockingQueueHandler(logging.handlers.QueueHandler):
    """
    Attempts to put record into queue without blocking.
    If queue is full and drop_on_full=True, the record is dropped (counter increments).
    """
    def __init__(self, q: queue.Queue, drop_on_full: bool = True):
        super().__init__(q)
        self.drop_on_full = drop_on_full
        self.dropped = 0
        self._lock = threading.Lock()

    # Preserve dict/list messages instead of stringifying them.
    # Python's default QueueHandler.prepare() flattens to string; we override it.
    def prepare(self, record: logging.LogRecord) -> logging.LogRecord:
        # Only collapse formatting if it's the classic str + args pattern
        if not isinstance(record.msg, (dict, list)):
            record.msg = record.getMessage()
            record.args = None
        # Keep exc_info for the formatter to render (stacktrace-friendly)
        return record

    def enqueue(self, record):
        try:
            # Non-blocking: never stall producers
            self.queue.put_nowait(record)
        except queue.Full:
            if self.drop_on_full:
                with self._lock:
                    self.dropped += 1
                # Do not log here to avoid recursion / hot-path work
            else:
                # Fallback: allow blocking (not recommended in hot loops)
                self.queue.put(record)


# ---- Setup / teardown API ----
class AsyncLoggerHandle:
    """Holds objects you may want to inspect or shutdown gracefully."""
    def __init__(
        self,
        queue_obj: queue.Queue,
        listener: logging.handlers.QueueListener,
        handler: NonBlockingQueueHandler,
    ):
        self.queue = queue_obj
        self.listener = listener
        self.handler = handler
        self._monitor_thread = None
        self._monitor_stop = threading.Event()
        self._stopped = False  # <-- add

    # -- Monitoring --
    def start_monitor(self, interval_s: float):
        if interval_s is None or interval_s <= 0:
            return
        def _monitor():
            while not self._monitor_stop.wait(interval_s):
                try:
                    qsize = self.queue.qsize()
                except Exception:
                    qsize = -1
                #print(f"[async-logger monitor] queue_size={qsize} dropped={self.handler.dropped}")
        t = threading.Thread(target=_monitor, daemon=True, name="async-logger-monitor")
        t.start()
        self._monitor_thread = t

    def stop_monitor(self):
        if self._monitor_thread:
            self._monitor_stop.set()
            self._monitor_thread.join(timeout=2.0)
            self._monitor_thread = None  # <-- add

    # -- Stats --
    def get_stats(self) -> dict:
        try:
            qsize = self.queue.qsize()
        except Exception:
            qsize = -1
        return {"queue_size": qsize, "dropped": self.handler.dropped}

    # -- Shutdown (idempotent) --
    def stop(self, timeout: float = 2.0):
        """Stop monitor then stop listener (flushes remaining records). Safe if called multiple times."""
        if self._stopped:
            return  # <-- guard: already stopped
        self._stopped = True

        self.stop_monitor()

        # If QueueListener was never started or already stopped, its _thread may be None.
        thread = getattr(self.listener, "_thread", None)
        if thread is None:
            return  # nothing to do

        try:
            # Python’s QueueListener.stop() joins the thread and sets _thread to None
            self.listener.stop()
        except Exception as e:
            # Best effort, don't raise during shutdown
            print("Warning: async logger listener.stop() failed:", e)

def _ensure_parent_dirs(path_str: str) -> str:
    p = Path(path_str)
    try:
        p.parent.mkdir(parents=True, exist_ok=True)
    except Exception as e:
        # Don't crash the app if directories can't be created; log to stderr later.
        print(f"Warning: could not create log directory '{p.parent}': {e}")
    return str(p)


def setup_async_logger(cfg: AsyncLoggerCfg) -> AsyncLoggerHandle:
    """
    Configure the root logger to use a background queue listener.
    Call once at program start. Returns a handle; call handle.stop() at exit
    (auto-called if cfg.register_atexit=True).
    """
    # Bounded queue to cap memory under logging storms
    q = queue.Queue(maxsize=cfg.queue_maxsize)

    # Producer-side: non-blocking queue handler
    qh = NonBlockingQueueHandler(q, drop_on_full=cfg.drop_on_full)
    qh.setLevel(cfg.level)

    # Select formatter (JSON or text)
    formatter: logging.Formatter = (
        JsonFormatter()
        if cfg.json_format
        else logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s")
    )

    # Consumer-side handlers (live on listener thread)
    logfile = _ensure_parent_dirs(cfg.logfile) if cfg.ensure_dirs else cfg.logfile
    file_handler = logging.handlers.RotatingFileHandler(
        logfile,
        maxBytes=cfg.max_bytes,
        backupCount=cfg.backup_count,
        delay=True,               # do not open file until first emit
        encoding="utf-8",
    )
    file_handler.setFormatter(formatter)
    file_handler.setLevel(cfg.level)

    handlers = [file_handler]

    if cfg.console:
        console = logging.StreamHandler()
        console.setFormatter(formatter)
        console.setLevel(cfg.level)
        handlers.append(console)

    # Listener consumes from queue and forwards to real handlers
    listener = logging.handlers.QueueListener(q, *handlers, respect_handler_level=True)
    listener.start()

    # Attach the queue handler to the root logger so all loggers inherit it
    root = logging.getLogger()
    root.setLevel(cfg.level)

    # Avoid duplicate installs if called twice
    if not any(isinstance(h, NonBlockingQueueHandler) for h in root.handlers):
        root.addHandler(qh)

    handle = AsyncLoggerHandle(q, listener, qh)

    if cfg.monitor_interval_s:
        handle.start_monitor(cfg.monitor_interval_s)

    if cfg.register_atexit:
        atexit.register(handle.stop)

    return handle
