"""
Luca Grigolin at PROFACTOR GmbH

Coordinator main and only code.

"""

########################## COORDINATOR NODE ##############################

import threading
import paho.mqtt.client as mqtt
import msgspec

MQTT_BROKER     = "localhost"
MQTT_PORT       = 1883
MQTT_CLIENT_ID  = "coordinator_client"
TOPIC_PLAN            = "coordinator/plan"
TOPIC_PLAN_TO_VISION       = "coordinator/plan_to_vision"
TOPIC_PLANNED_TRAJECTS  = "planner/trajectories"
TOPIC_TERMINATION       = "coordinator/termination"
TOPIC_EXECUTION       = "coordinator/execution"
TOPIC_CALIBRATION       = "coordinator/calibration_trigger"
TOPIC_VISION = "coordinator/vision"
TOPIC_PICKING_POSE = "vision/picking_pose"
TOPIC_FEEDBACK = "executor/feedback"
TOPIC_CALIB_TERM = "vision/calib_termination"
TOPIC_PLANNER_READY = "planner/ready"
TOPIC_EST_EXT = "planner/estimate_ext"
TOPIC_SAVE_POSE = "coordinator/save_pose"

MQTT_QOS                = 1 # At least once delivery

# --------- Shared elements ----------
plan_blocking_event = threading.Event()  # to block the thread until a trajectory is received
execution_blocking_event = threading.Event()  # to block the thread until an execution feedback is received
vision_blocking_event = threading.Event()  # to block the thread until a picking pose is received
calibration_blocking_event = threading.Event()  # to block the thread until robot needs to get back to Vision pose
mqtt_connected_event = threading.Event() # to block the thread until mqtt communication is ready
planner_ready_event = threading.Event() # to block the thread until the robot is at vision pose 
failed_routine_event = threading.Event() # event that represents if a robot routine failed
encoder = msgspec.json.Encoder() # pose encoder for serialization
lock = threading.Lock()
trajectory_timeout = 5.0
execution_feedback_timeout = 25.0
vision_detection_timeout = 300.0 
saving_pose_timeout = 2.0

# --------- typed struct to encode the Ply ID ----------
class plan(msgspec.Struct):
    ply_id: str
    path: list[str]
    travel_height: float = 1.5  # default travel height
    approach: float = 0.2       # default placing approach distance
    speed: float = 0.3          # default speed m/s

# --------- MQTT callbacks ----------
def on_connect(client, userdata, flags, reason_code, properties):
    print("[MQTT] Connected, reason:", reason_code)
    mqtt_connected_event.set()  # signal that MQTT is connected
    # Subscribe to the topics
    client.subscribe(TOPIC_PLANNED_TRAJECTS, qos=MQTT_QOS)
    client.subscribe(TOPIC_PICKING_POSE, qos=MQTT_QOS)
    client.subscribe(TOPIC_FEEDBACK, qos=MQTT_QOS)
    client.subscribe(TOPIC_PLANNER_READY, qos=MQTT_QOS)
    client.subscribe(TOPIC_EST_EXT, qos=MQTT_QOS)


def on_message(client, userdata, msg):
    # aknowledging planner ready message
    if msg.topic == TOPIC_PLANNER_READY:
        planner_ready_event.set()

    # aknowledging calibration process is finished
    if msg.topic == TOPIC_EST_EXT:
        print(f"[MQTT] Estimating Extrinsics while getting to Vision Pose.")
        calibration_blocking_event.set()

    # aknowledging estimated picking pose
    if msg.topic == TOPIC_PICKING_POSE:
        payload = msg.payload.decode("utf-8")
        if not payload:
            print("[ERROR] Received empty picking pose !")
            return
        # else continue with the execution
        #print(f"[MQTT] Received picking pose on '{TOPIC_PICKING_POSE}'")
        vision_blocking_event.set()

    # aknowledging published trajectory
    if msg.topic == TOPIC_PLANNED_TRAJECTS:
        payload = msg.payload.decode("utf-8")
        if not payload:
            print("[ERROR] Received empty trajectory !")
            return
        # continue with the execution
        #print(f"[MQTT] Received trajectory on '{TOPIC_PLANNED_TRAJECTS}'")
        plan_blocking_event.set()

    # aknowledging published execution feedback
    if msg.topic == TOPIC_FEEDBACK:
        # continue with the execution
        #print(f"[MQTT] Received execution feedback on '{TOPIC_FEEDBACK}'")
        execution_blocking_event.set()
    



# --------- Create MQTT client ----------
def create_mqtt_client():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=MQTT_CLIENT_ID)
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    client.loop_start()  # run network loop in background
    return client

# --------- Publish Plan via MQTT ----------
def publish_plan(client, path, this_ply_id, travel_height, approach, speed):
    pose = plan(
        ply_id= this_ply_id,
        path=path,
        travel_height=travel_height,
        approach=approach,
        speed=speed
    )
    payload = encoder.encode(pose)  # bytes
    result = client.publish(TOPIC_PLAN, payload=payload, qos=MQTT_QOS)
    rc,mid = result
    if not rc == mqtt.MQTT_ERR_SUCCESS:
        #print(f"[MQTT] Published plan successfully.")
    # else:
        print(f"[MQTT] Failed to publish plan. Error code: {rc}")

# --------- Publish Plan To Vision via MQTT ----------
def publish_plan_to_vision(client):
    result = client.publish(TOPIC_PLAN_TO_VISION, payload="", qos=MQTT_QOS)
    rc,mid = result
    if not rc == mqtt.MQTT_ERR_SUCCESS:
        #print(f"[MQTT] Published plan To vision successfully.")
    # else:
        print(f"[MQTT] Failed to publish Plan To Vision. Error code: {rc}")

# --------- Publish detection via MQTT ----------
def picking_pose_estimation(client, ply_id):
    # Clear previous event 
    vision_blocking_event.clear()
    result = client.publish(TOPIC_VISION, payload=ply_id, qos=MQTT_QOS)
    rc, mid = result
    if not rc == mqtt.MQTT_ERR_SUCCESS:
        #print("[MQTT] Published detect trigger for: " + ply_id)
    # else:
        print(f"[MQTT] Failed to publish detect trigger for: " + ply_id + ". Error: {rc}")
    if not vision_blocking_event.wait(vision_detection_timeout):
        print("[MQTT] Vision failed to estimate the " + ply_id + f" optimal picking pose in {vision_detection_timeout} seconds.")
        failed_routine_event.set()


# --------- Publish execution trigger via MQTT ----------
def publish_execution(client):
    result = client.publish(TOPIC_EXECUTION, payload="", qos=MQTT_QOS)
    rc,mid = result
    if not rc == mqtt.MQTT_ERR_SUCCESS:
        #print(f"[MQTT] Published execution successfully.")
    # else:
        print(f"[MQTT] Failed to publish execution. Error code: {rc}")

# --------- Publish save Pose trigger via MQTT ----------
def publish_save_pose(client, savePose_ply_id):
    result = client.publish(TOPIC_SAVE_POSE, payload=savePose_ply_id, qos=MQTT_QOS)
    rc,mid = result
    if not rc == mqtt.MQTT_ERR_SUCCESS:
        #print(f"[MQTT] Published execution successfully.")
    # else:
        print(f"[MQTT] Failed to publish execution. Error code: {rc}")

# --------- Publish termination via MQTT ----------
def publish_termination(client):
    result = client.publish(TOPIC_TERMINATION, payload="", qos=MQTT_QOS)
    rc,mid = result
    if not rc == mqtt.MQTT_ERR_SUCCESS:
        #print(f"[MQTT] Published termination successfully.")
    # else:
        print(f"[MQTT] Failed to publish termination. Error code: {rc}")

# --------- Publish calibration via MQTT ----------
def publish_calibration_trigger(client):
    result = client.publish(TOPIC_CALIBRATION, payload="", qos=MQTT_QOS)
    rc,mid = result
    if not rc == mqtt.MQTT_ERR_SUCCESS:
        #print(f"[MQTT] Published calibration trigger successfully.")
    # else:
        print(f"[MQTT] Failed to publish calibration. Error code: {rc}")

# --------- Plan and execute Loop ----------
def plan_execute(
    mqtt_client,
    path=None,
    this_ply_id=None,
    travel_height=None,
    approach=None,
    speed=None,
    *,
    to_vision=False
):
    # don't plan execute on the same routine if the routine has failed
    if not failed_routine_event.is_set():
        # Clear previous events
        plan_blocking_event.clear()
        execution_blocking_event.clear()

        # publish the plan
        if to_vision: 
            #print("\nRobot getting to Vision Pose ...")
            publish_plan_to_vision(mqtt_client)
        else: 
            publish_plan(mqtt_client, path, this_ply_id, travel_height, approach, speed)
        
        #print("[MQTT] Waiting for a trajectory ...")

        # Wait for the trajectory
        if plan_blocking_event.wait(timeout=trajectory_timeout):

            # publish the execution
            publish_execution(mqtt_client)

            #print("[MQTT] Waiting for an execution feedback ...")

            # Wait for a feedback
            if not execution_blocking_event.wait(timeout=execution_feedback_timeout):
                print(f"[MQTT] waited for an execution feedback from planner for more than {execution_feedback_timeout} seconds")
                failed_routine_event.set()
            if to_vision:
                    print("\nRobot at Vision Pose. Ready for next command.")
        else:
            print(f"[MQTT] waited for a trajectory from planner for more than {trajectory_timeout} seconds")
            failed_routine_event.set()

# --------- Print UI controls on terminal  ----------
def print_ui_controls():
    print(
        "\n=============== UI Controls ===============\n"
        "  [q] or [ESC] : Quit application\n"
        "  [c]          : Start Extrinsic Calibration\n"
        "  [s]          : Save Current Robot Pose\n"
        "  [plyID]      : Initiate Draping Routine\n"
        "===========================================\n"
    )

# --------- Print UI continue or kill option  ----------
def wait_for_confirm(msg):
    if not failed_routine_event.is_set():
        user_input = input(
            f"\n[CONFIRM] {msg} — press ENTER to continue, 'a' to abort: "
        ).strip().lower()
        if user_input == "a":
            failed_routine_event.set()

# --------- Main Coordinator Loop ----------
def run_coordinator():
    mqtt_client = create_mqtt_client()
    mqtt_connected_event.wait()   # block until mqtt is connected
    print(f"[MQTT] System initialization ...")
    planner_ready_event.wait() # block until planner is fully initialized
    print(f"[MQTT] System is fully initialized.")
    planner_ready_event.clear() # reset for next 

    wait_for_confirm("\nMoving to Vision Pose\n")
    plan_execute(mqtt_client, to_vision=True)
    if failed_routine_event.is_set():
        print("\n[!] BACK TO VISION PATH IS COLLIDING [!]")
        print("\n[!] Please, jog the robot in a safe pose [!]\n")
        input(
            f"[CONFIRM] Return to VISION\n — press ENTER to continue: "
        ).strip()
        plan_execute(mqtt_client, to_vision=True)

    try:
        while True:

            print_ui_controls()
            this_ply_id = input("\nEnter Next Command: ").strip().lower()

            if this_ply_id.lower() in ("q", "quit", "exit"):
                print("\n[TERMINATION] Termination commanded.")
                publish_termination(mqtt_client)
                break
            
            if this_ply_id.lower() in ("s"):
                print("\nEnter [plyID] of the ply to place at this Robot Pose ")
                savePose_ply_id = input("(or enter [vision] to reset the vision pose): ").strip().lower()
                publish_save_pose(mqtt_client, savePose_ply_id)
                print("\n")
                if planner_ready_event.wait(saving_pose_timeout):
                    print(savePose_ply_id + " Pose was succesfully saved in memory.")
                else:
                    print("[ERROR]: " + savePose_ply_id + " Pose wasn't saved in memory!")
                planner_ready_event.clear()
                continue

            if this_ply_id.lower() in ("c"):
                print("\n[CALIBRATION] Performing Extrinsics Calibration ... \n")
                publish_calibration_trigger(mqtt_client)

                # wait for the planner to be ready after calibration loop
                calibration_blocking_event.wait()

                calibration_blocking_event.clear() # reset
                plan_execute(mqtt_client, to_vision=True)


            
            else: # add if checking the ply Id name

                # set process global routine parameters
                travel_height = 1.5 # m
                approach = 0.2 # m
                travel_speed = 0.3 # percentage
                approach_speed = 0.1 # percentage

                # ASSUMPTION: TCP at VisionPose

                # trigger Vision 
                wait_for_confirm("\nTrigger vision and Approach estimated PICKING\n")
                picking_pose_estimation(mqtt_client, this_ply_id)

                # approach PICKING 
                path = [
                    "visionPose",
                    "TableTopPose"
                ]
                plan_execute(mqtt_client, path, this_ply_id, travel_height, approach, travel_speed)

                # PICKING 
                wait_for_confirm("PICKING (table → picking)\n")
                path = [
                    "TableTopPose",
                    "PickingPose"
                ]
                plan_execute(mqtt_client, path, this_ply_id, travel_height, approach, approach_speed)

                # approach PLACING 
                wait_for_confirm("Approach PLACING\n")
                path = [
                    "PickingPose",
                    "TableTopPose",
                    "MouldTopPose",
                    this_ply_id + "TopPose"
                ]
                plan_execute(mqtt_client, path, this_ply_id, travel_height, approach, travel_speed)

                wait_for_confirm("PLACING\n")
                # PLACING 
                path = [
                    this_ply_id + "TopPose",
                    this_ply_id + "PlacingPose"
                ]
                plan_execute(mqtt_client, path, this_ply_id, travel_height, approach, approach_speed)

                wait_for_confirm("Return to VISION\n")
                # TO VISION 
                path = [
                    this_ply_id + "PlacingPose",
                    this_ply_id + "TopPose",
                    "MouldTopPose",
                    "visionPose"
                    ]
                plan_execute(mqtt_client, path, this_ply_id, travel_height, approach, travel_speed)

                if failed_routine_event.is_set():
                    print("The current routine has terminated early.")
                    failed_routine_event.clear() # reset 
                    input(
                        f"\n[CONFIRM] Return to VISION\n — press ENTER to continue: "
                    ).strip()
                    plan_execute(mqtt_client, to_vision=True)
                    if failed_routine_event.is_set():
                        print("\n[!] BACK TO VISION PATH IS COLLIDING [!]")
                        print("\n[!] Please, jog the robot in a safe pose [!]\n")
                        input(
                            f"[CONFIRM] Return to VISION\n — press ENTER to continue: "
                        ).strip()
                        plan_execute(mqtt_client, to_vision=True)

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt, shutting down coordinator.")
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()


if __name__ == "__main__":
    run_coordinator()
