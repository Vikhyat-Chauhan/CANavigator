import sys

# ---- Imports for Gazebo Transport & Msgs ----
# Requires: sudo apt install python3-gz-transport13 python3-gz-msgs10
try:
    from gz.transport13 import Node, AdvertiseMessageOptions  # type: ignore
except Exception:
    print("ERROR: Install 'python3-gz-transport13'.", file=sys.stderr)
    raise

Twist = None
Vector3d = None
_errors = []
for base in ("gz.msgs10", "gz.msgs", "ignition.msgs"):
    try:
        Twist = __import__(f"{base}.twist_pb2", fromlist=["Twist"]).Twist
        Vector3d = __import__(f"{base}.vector3d_pb2", fromlist=["Vector3d"]).Vector3d
        print(f"[hydra] Using msgs from: {base}")
        break
    except Exception as e:
        _errors.append((base, str(e)))

if Twist is None or Vector3d is None:
    print("ERROR: Could not import gz/ignition msgs (Twist, Vector3d).", file=sys.stderr)
    print("Tried:", _errors, file=sys.stderr)
    print("Install 'python3-gz-msgs10' (recommended).", file=sys.stderr)
    sys.exit(2)

class GzVelPub:
    """
    Minimal, fast velocity publisher:
    - Persistent Node & Publisher
    - Reuses a pre-allocated Twist protobuf (low GC pressure)
    """
    def __init__(self, topic: str, rate_limit_hz: float | None = None):
        self.topic = topic
        self.node = Node()
        self._pub = None
        self._publish_fn = None

        # Try modern API: node.advertise(...).publish(msg)
        try:
            if rate_limit_hz:
                opts = AdvertiseMessageOptions()
                opts.msgs_per_sec = float(rate_limit_hz)
                self._pub = self.node.advertise(self.topic, Twist, opts)  # type: ignore[attr-defined]
            else:
                self._pub = self.node.advertise(self.topic, Twist)  # type: ignore[attr-defined]
            if not self._pub:
                raise RuntimeError("node.advertise returned None")
            self._publish_fn = lambda msg: self._pub.publish(msg)  # type: ignore[union-attr]
            print("[hydra] Transport API: node.advertise(...).publish(msg)")
        except Exception:
            # Fallback legacy: Node.Advertise + Node.Publish
            ok = False
            try:
                ok = self.node.Advertise(self.topic, Twist)  # type: ignore[attr-defined]
            except Exception:
                pass
            if not ok:
                raise RuntimeError(f"Failed to advertise publisher on '{self.topic}' via any API.")
            self._publish_fn = lambda msg: self.node.Publish(self.topic, msg)  # type: ignore[attr-defined]
            print("[hydra] Transport API: Node.Advertise(...), Node.Publish(topic, msg)")

        self._msg = Twist()

    def send(self, lin_xyz: tuple[float, float, float], ang_zyx: tuple[float, float, float] = (0.0, 0.0, 0.0)):
        lx, ly, lz = lin_xyz
        ax, ay, az = ang_zyx
        self._msg.linear.CopyFrom(Vector3d(x=lx, y=ly, z=lz))
        self._msg.angular.CopyFrom(Vector3d(x=ax, y=ay, z=az))
        ok = self._publish_fn(self._msg)
        if ok is False:
            raise RuntimeError("Publish failed")
