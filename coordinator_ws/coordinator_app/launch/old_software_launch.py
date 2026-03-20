"""
Luca Grigolin at PROFACTOR GmbH

Old Launch for the BioStruct_DrapeBot software.

Requires the RWS-EGM three launches prior

cd coordinator_app/launch 
./old_software_launch.py 



"""

import subprocess
import os
import time

HOME = os.path.expanduser("~")

processes = []
failed = []

def launch_terminal(title, command):
    try:
        p = subprocess.Popen([
            "gnome-terminal",
            "--title", title,
            "--",
            "bash",
            "-c",
            f"{command}; exec bash"
        ])
        processes.append((title, p))
    except Exception as e:
        failed.append((title, str(e)))

# # TERMINAL 1 for real hardware
# launch_terminal(
#     "RWS",
#     f"""
#     cd {HOME}/BioStruct_DrapeBot/abb_ws &&
#     ros2 launch abb_workcell_bringup abb_rws_client.launch.py robot_ip:=192.168.125.1
#     """
# )
# # TERMINAL 1 for virtual hardware
# launch_terminal(
#     "RWS",
#     f"""
#     cd {HOME}/BioStruct_DrapeBot/abb_ws &&
#     ros2 launch abb_workcell_bringup abb_rws_client.launch.py 
#     """
# )

# # TERMINAL 2 for real hardware
# launch_terminal(
#     "ABB control",
#     f"""
#     cd {HOME}/BioStruct_DrapeBot/abb_ws &&
#     ros2 launch abb_workcell_bringup abb_control.launch.py rws_ip:=192.168.125.1
#     """
# )
# # TERMINAL 2 for virtual hardwarecv2
# launch_terminal(
#     "ABB control",
#     f"""
#     cd {HOME}/BioStruct_DrapeBot/abb_ws &&
#     ros2 launch abb_workcell_bringup abb_control.launch.py 
#     """
# )
# # TERMINAL 2 for fake hardware
# launch_terminal(
#     "ABB control",
#     f"""
#     cd {HOME}/BioStruct_DrapeBot/abb_ws &&
#     ros2 launch abb_workcell_bringup abb_control.launch.py use_fake_hardware:=true
#     """
# )

# # TERMINAL 3
# launch_terminal(
#     "EGM trigger",
#     f"""
#     cd {HOME}/BioStruct_DrapeBot/abb_ws &&
#     ros2 service call /rws_client/start_egm_joint abb_robot_msgs/srv/TriggerWithResultCode
#     """
# )

# TERMINAL 4
launch_terminal(
    "Move Group",
    f"""
    cd {HOME}/BioStruct_DrapeBot/abb_ws &&
    ros2 launch biostruct_robotics_lab move_group.launch.py
    """
)

# TERMINAL 5
launch_terminal(
    "Planner",
    f"""
    cd {HOME}/BioStruct_DrapeBot/abb_ws &&
    ros2 launch biostruct_robotics_lab planning_node.launch.py
    """
)

# TERMINAL 6
launch_terminal(
    "Executor",
    f"""
    cd {HOME}/BioStruct_DrapeBot/abb_ws &&
    ros2 launch biostruct_robotics_lab execution_node.launch.py
    """
)

# TERMINAL 7
launch_terminal(
    "Vision Node",
    f"""
    cd {HOME}/BioStruct_DrapeBot/vision_ws/vision_app &&
    source .vision_env/bin/activate &&
    python src/vision_node.py
    """
)

# TERMINAL 8
launch_terminal(
    "Coordinator",
    f"""
    cd {HOME}/BioStruct_DrapeBot/coordinator_ws/coordinator_app &&
    source .coordinator_env/bin/activate &&
    python src/coordinator.py
    """
)

# ---- check launch status ----
time.sleep(1)  # give terminals time to start

for title, p in processes:
    rc = p.poll()
    # gnome-terminal  exits immediately with 0 even when it successfully opened a window
    if rc is not None and rc != 0:
        failed.append((title, f"gnome-terminal exited immediately with code {rc}"))


# ---- launch report ----

if failed:
    print("⚠️ Some components failed to launch:")
    for title, reason in failed:
        print(f"  - {title}: {reason}")
else:
    print("✅ All components launched successfully")
