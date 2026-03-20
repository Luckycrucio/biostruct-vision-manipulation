"""
Luca Grigolin at PROFACTOR GmbH

Launch BioStruct_DrapeBot software components, each in a separate terminal.

Usage:
    python3 launch_stack.py --robot-ip 192.168.125.1 (Real or Virtual Hardware)
    python3 launch_stack.py                          (Fake Hardware)
"""

import os
import time
import subprocess
import argparse

HOME = os.path.expanduser("~")


def term(title: str, cmd: str) -> None:
    """Open a terminal that stays open after the command finishes."""
    subprocess.Popen([
        "gnome-terminal", "--title", title, "--",
        "bash", "-lc", cmd + "; exec bash"
    ])


def term_wait_close(title: str, cmd: str) -> None:
    """
    Open a terminal that closes when the command finishes, and BLOCK until it closes.
    Requires gnome-terminal supporting '--wait' (common on modern GNOME Terminal).
    """
    p = subprocess.Popen([
        "gnome-terminal", "--wait", "--title", title, "--",
        "bash", "-lc", cmd
    ])
    p.wait()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-ip", default=None)
    args = ap.parse_args()

    abb_ws = f"{HOME}/BioStruct_DrapeBot/abb_ws"
    vision_app = f"{HOME}/BioStruct_DrapeBot/vision_ws/vision_app"
    coord_app = f"{HOME}/BioStruct_DrapeBot/coordinator_ws/coordinator_app"

    if args.robot_ip:
        # 1) Start RWS client
        term(
            "ABB RWS Client",
            f"cd {abb_ws} && ros2 launch abb_workcell_bringup abb_rws_client.launch.py robot_ip:={args.robot_ip}"
        )

        # Wait enough
        time.sleep(2)

        # 2) Start ABB control
        term(
            "ABB control",
            f"cd {abb_ws} && ros2 launch abb_workcell_bringup abb_control.launch.py rws_ip:={args.robot_ip}"
        )

        # Wait enough
        time.sleep(2)

        # 3) Call start_egm_joint; wait until this terminal finishes cleanly and closes
        term_wait_close(
            "Start EGM Joint",
            f"cd {abb_ws} && ros2 service call "
            f"/rws_client/start_egm_joint abb_robot_msgs/srv/TriggerWithResultCode"
        )

    else:
        term(
            "ABB control",
            f"cd {abb_ws} && ros2 launch abb_workcell_bringup abb_control.launch.py use_fake_hardware:=true"
        )

    # Continue with the rest only after the EGM service call terminal has finished (robot-ip case)

    # DEVELOPMENT VERSION (PROFACTOR ROBOTICS LAB)
    # term("Move Group", f"cd {abb_ws} && ros2 launch biostruct_robotics_lab move_group.launch.py")
    # term("Planner",    f"cd {abb_ws} && ros2 launch biostruct_robotics_lab planning_node.launch.py")
    # term("Executor",   f"cd {abb_ws} && ros2 launch biostruct_robotics_lab execution_node.launch.py")

    #  AMURA SCENE
    term("Move Group", f"cd {abb_ws} && ros2 launch biostruct_amura move_group.launch.py")
    term("Planner",    f"cd {abb_ws} && ros2 launch biostruct_amura planning_node.launch.py")
    term("Executor",   f"cd {abb_ws} && ros2 launch biostruct_amura execution_node.launch.py")

    term(
        "Vision Node",
        f"cd {vision_app} && source .vision_env/bin/activate && python src/vision_node.py"
    )

    term(
        "Coordinator",
        f"cd {coord_app} && source .coordinator_env/bin/activate && python src/coordinator.py"
    )


if __name__ == "__main__":
    main()
