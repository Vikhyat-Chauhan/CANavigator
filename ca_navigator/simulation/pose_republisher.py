"""
A bit of History:
I tried several methods to get drone positions when the simulation is running :
1. I tried the pose publisher plugin with ros bridge with the drone [/model/drone1/pose]
    ->  When I looked at the entries the drone1 kept publishing the the information about the links in the drone1 model not the drone itself. Which was
        impossible to resolve nomatter what config I used in the pose publisher plugin.
2. I tried to use pose publisher for the world, but it was not compatible with the world and was throwing error when loading the world sdf file.
3. I tried to create a ros bridge from the world pose publisher [/world/airport/pose/info] to ros topic
    ->  The problem I ran across was that a bridge from gz pose vector to ros removes all essential information 
        about the entity (like drone1) and give some output thats gibberish to parse for positions. This is apparantly because ros converts the gz.msgs.Pose to something native to ros.
[Current hotfix]
4. Finally I had to create a script that uses the cli command of gz to get the pose infromation about all the entitites in the world and parse them with a python code to then create
    a ros bridge from which I can later fetch this information, I am guessing it might not be very efficient but this is the only option i have now.
"""
#!/usr/bin/env python3
# ca_navigator/pose_republisher.py
import json, subprocess, threading
from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Sequence, Union, List
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped

from ..config import TeleopConfig


# ---------- Helpers ----------
def _fnum(x, default=0.0):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


def _as_entity_list(entities: Union[str, Iterable[str]]) -> List[str]:
    if isinstance(entities, str):
        return [entities]
    return [str(e) for e in entities]


@dataclass
class PoseBridgeConfig:
    world_pose_topic: str = TeleopConfig.world_pose_topic  # gz Pose_V stream
    out_template: str = "/model/{entity}/pose/info"        # per-entity ROS2 topic


# ---------- Multi-entity republisher (one reader, many pubs) ----------
class MultiWorldPoseRepublisher(Node):
    """
    Reads Gazebo Pose_V via `gz topic -e --json-output` ONCE and republishes
    PoseStamped for each requested entity to `/model/<entity>/pose/info`.
    Exact name match only (no 'entity::link' allowed).

    This node DOES NOT spin; attach it to your shared executor.
    """
    def __init__(
        self,
        cfg: PoseBridgeConfig,
        entities: Sequence[str],
        *,
        callback_group=None,
    ):
        super().__init__("world_pose_multi_republisher")
        self._cfg = cfg
        self._entities = sorted(set(entities))
        self._stop = threading.Event()
        self._proc: Optional[subprocess.Popen] = None
        self._cbg = callback_group or ReentrantCallbackGroup()

        # Create a publisher per entity
        self._pubs: Dict[str, any] = {}
        for name in self._entities:
            topic = cfg.out_template.format(entity=name)
            self._pubs[name] = self.create_publisher(PoseStamped, topic, 10)
            self.get_logger().info(
                f"[pose-repub] {cfg.world_pose_topic} → {topic} (entity='{name}')"
            )

        # Background reader thread (not a ROS spin; just reads CLI)
        self._reader_thread = threading.Thread(target=self._reader, daemon=True)
        self._reader_thread.start()

    # ---- Reader ----
    def _reader(self):
        cmd = ["gz", "topic", "-e", "-t", self._cfg.world_pose_topic, "--json-output"]
        try:
            self._proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, text=True, bufsize=1
            )
        except FileNotFoundError:
            self.get_logger().error("`gz` not found in PATH.")
            return

        names_exact = set(self._entities)
        stdout = self._proc.stdout

        if stdout is None:
            self.get_logger().error("Failed to open `gz topic` stdout.")
            return

        for line in stdout:
            if self._stop.is_set():
                break
            line = (line or "").strip()
            if not line or not line.startswith("{"):
                continue

            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue

            poses = obj.get("pose")
            if isinstance(poses, list):
                # Envelope case: Pose_V { pose: [...] }
                for p in poses:
                    nm = str(p.get("name", ""))
                    if nm in names_exact:
                        self._publish_pose(nm, p)
            else:
                # Per-pose line case
                nm = str(obj.get("name", ""))
                if nm in names_exact:
                    self._publish_pose(nm, obj)

    def _publish_pose(self, entity: str, p: dict):
        pos = p.get("position", {}) or {}
        ori = p.get("orientation", {}) or {}
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = _fnum(pos.get("x"))
        msg.pose.position.y = _fnum(pos.get("y"))
        msg.pose.position.z = _fnum(pos.get("z"))
        msg.pose.orientation.w = _fnum(ori.get("w"), 1.0)
        msg.pose.orientation.x = _fnum(ori.get("x"))
        msg.pose.orientation.y = _fnum(ori.get("y"))
        msg.pose.orientation.z = _fnum(ori.get("z"))

        pub = self._pubs.get(entity)
        if pub:
            pub.publish(msg)

    # ---- Lifecycle ----
    def stop(self):
        """Stop the CLI reader thread and terminate the `gz topic` process."""
        self._stop.set()
        try:
            if self._proc and self._proc.poll() is None:
                self._proc.terminate()
        except Exception:
            pass
        if getattr(self, "_reader_thread", None):
            self._reader_thread.join(timeout=0.5)

    def close(self):
        """Stop and destroy this node. Caller should also remove from executor."""
        try:
            self.stop()
        finally:
            try:
                self.destroy_node()
            except Exception:
                pass


# ---------- Factories (no spin, no executor ownership) ----------
def create_pose_republisher(
    entities: Union[str, Iterable[str]],
    world_pose_topic: str = TeleopConfig.world_pose_topic,
    out_template: str = "/model/{entity}/pose/info",
    *,
    callback_group=None,
) -> MultiWorldPoseRepublisher:
    """
    Construct the republisher node. Caller must add it to an executor and spin elsewhere.
    """
    ent_list = _as_entity_list(entities)
    node = MultiWorldPoseRepublisher(
        PoseBridgeConfig(world_pose_topic=world_pose_topic, out_template=out_template),
        entities=ent_list,
        callback_group=callback_group,
    )
    return node


def add_pose_republisher_to_executor(
    executor,
    entities: Union[str, Iterable[str]],
    world_pose_topic: str = TeleopConfig.world_pose_topic,
    out_template: str = "/model/{entity}/pose/info",
    *,
    callback_group=None,
) -> MultiWorldPoseRepublisher:
    """
    Convenience helper: build node and add to the provided executor.
    Does NOT spin. The caller owns the single spin loop.
    """
    node = create_pose_republisher(
        entities,
        world_pose_topic=world_pose_topic,
        out_template=out_template,
        callback_group=callback_group,
    )
    executor.add_node(node)
    return node
