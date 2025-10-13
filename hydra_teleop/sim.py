import os, signal, subprocess, time
from .config import TeleopConfig


def launch(cmd: list[str] | tuple[str, ...], env: dict | None = None) -> subprocess.Popen:
    """Start a process in a new process group so we can kill the whole tree later."""
    return subprocess.Popen(cmd, env=env, start_new_session=True)

def kill_process_tree(proc: subprocess.Popen | None, _name: str = "", gentle_timeout=5, term_timeout=5) -> None:
    """SIGINT -> SIGTERM -> SIGKILL on the process group."""
    if not proc or proc.poll() is not None:
        return
    pgid = proc.pid
    try:
        os.killpg(pgid, signal.SIGINT)
        proc.wait(timeout=gentle_timeout)
    except Exception:
        pass
    if proc.poll() is None:
        try:
            os.killpg(pgid, signal.SIGTERM)
            proc.wait(timeout=term_timeout)
        except Exception:
            pass
    if proc.poll() is None:
        try:
            os.killpg(pgid, signal.SIGKILL)
        except Exception:
            pass

def start_sim(cfg: TeleopConfig) -> subprocess.Popen | None:
    """
    Launch Gazebo and return the Popen handle immediately (non-blocking).
    If cfg.launch_sim is False, returns None.
    """
    if not cfg.launch_sim:
        #print("[start_sim] launch_sim=False → not starting Gazebo.")
        return None

    sim_cmd = [*cfg.sim_cmd, cfg.world_path, "--render-engine", "ogre2"]
    #print(f"[start_sim] Launching: {' '.join(sim_cmd)}")
    sim = launch(sim_cmd, env=cfg.sim_env)
    if cfg.sim_boot_secs and cfg.sim_boot_secs > 0:
        time.sleep(cfg.sim_boot_secs)   # give Gazebo time to load
    return sim

def stop_sim(sim: subprocess.Popen | None) -> None:
    """Tear down the Gazebo process tree if it was started."""
    kill_process_tree(sim, "gz_sim")
