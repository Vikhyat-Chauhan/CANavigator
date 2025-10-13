# config.py
from dataclasses import dataclass
import os

@dataclass
class TeleopConfig:
    # --- Sim / world ---
    world_path: str = "/home/vikhyat-chauhan/Documents/Hydra/worlds/airport_world.sdf"
    sim_cmd: tuple[str, ...] = ("gz", "sim", "-r")  # world appended at runtime : this is GUI version
    #sim_cmd: tuple[str, ...] = ("gz", "sim", "4", "-s", "-r")  # world appended at runtime : this is headless version
    sim_env: dict | None = None
    sim_boot_secs: float = 8.0
    launch_sim: bool = True

    # --- Gazebo transport ---
    topic: str = "/model/drone1/cmd_vel"  # gz.msgs.Twist
    world_pose_topic: str = "/world/airport/pose/info"

    # --- Teleop dynamics ---
    rate_hz: float = 30.0
    speed_x: float = 5.0
    speed_y: float = 5.0
    speed_z: float = 5.0
    yaw_rate: float = 0.8

    # --- Logging ---
    log_to_csv: bool = False          # set False if you only want console output
    log_dir: str = "logs"            # directory for CSVs
    log_name: str = "teleop"         # filename prefix
    log_headers: list[str] | None = None  # optional fixed columns

    def __post_init__(self):
        if self.sim_env is None:
            self.sim_env = {
                **os.environ,
                "QT_QPA_PLATFORM": "xcb",
                "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
                "__EGL_VENDOR_LIBRARY_FILENAMES": "/usr/share/glvnd/egl_vendor.d/10_nvidia.json",
            }
