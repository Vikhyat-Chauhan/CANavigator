import os, signal, subprocess, time
from ..config import TeleopConfig


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
    sim_cmd = [*cfg.sim_cmd, cfg.world_path, "--render-engine", "ogre2", "--headless-rendering"]
    sim = launch(sim_cmd, env=cfg.sim_env)
    if cfg.sim_boot_secs and cfg.sim_boot_secs > 0:
        time.sleep(cfg.sim_boot_secs)   # give Gazebo time to load
    return sim

def reset_sim(sim: subprocess.Popen | None, cfg: TeleopConfig) -> None:
    """
    Teleport the drone back to its start pose between strategies.
    Avoids a full Gazebo restart — the same process keeps running.
    """
    if sim is None or sim.poll() is not None:
        return
    # Extract world name from "/world/{name}/pose/info"
    parts = cfg.world_pose_topic.split("/")
    world_name = parts[2] if len(parts) > 2 else "airport"
    pose_req = (
        f'name: "drone1" '
        f'position: {{x: {cfg.start_x} y: {cfg.start_y} z: {cfg.start_z}}} '
        f'orientation: {{w: 1.0 x: 0.0 y: 0.0 z: 0.0}}'
    )
    try:
        subprocess.run(
            [
                "gz", "service",
                "-s", f"/world/{world_name}/set_pose",
                "--reqtype", "gz.msgs.Pose",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "2000",
                "--req", pose_req,
            ],
            timeout=5,
            capture_output=True,
        )
    except Exception:
        pass
    time.sleep(1.0)  # let physics settle after teleport


def stop_sim(sim: subprocess.Popen | None) -> None:
    """Tear down the Gazebo process tree if it was started."""
    kill_process_tree(sim, "gz_sim")
    time.sleep(2)
