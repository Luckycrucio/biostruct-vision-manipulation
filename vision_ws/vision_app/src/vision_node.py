"""
Luca Grigolin at PROFACTOR GmbH

Vision main and only code, contains:

- Camera-TCP Extrinsic Calibration Algorithm
- Ply verification Algorithm
- 6D Ply-Pose Estimation Algorithm 

"""

########################## VISION NODE ##############################

from harvesters.core import Harvester
import numpy as np
import cv2
import time
import apriltag
from ultralytics import YOLO
import os
import glob
import json
import paho.mqtt.client as mqtt
import msgspec
import threading
from datetime import datetime
from scipy.spatial import cKDTree
from skimage.morphology import skeletonize 
from typing import Optional, Tuple, Dict
import random

MQTT_BROKER = "localhost"   # from tcp://localhost:1883
MQTT_PORT = 1883
MQTT_CLIENT_ID = "vision_client"
TOPIC_PICKING_POSE  = "vision/picking_pose"
TOPIC_TERMINATION = "coordinator/termination"
TOPIC_TCP_TRANS = "planner/tcp_transform"
TOPIC_DETECT = "coordinator/vision"
TOPIC_EST_EXT = "planner/estimate_ext"
TOPIC_CALIBRATION  = "coordinator/calibration_trigger"
TOPIC_CALIB_DONE_VISION  = "vision/calib_done_vision"
MQTT_QOS = 1
CTI = "/opt/spinnaker/lib/spinnaker-gentl/Spinnaker_GenTL.cti"
CAMERA_INTRINSICS_FILE = "genie_nano_m2590_mono_calib_2742.yaml"
CAMERA_EXTRINSICS_FILE = "T_cam2tcp_10_00_29__03022026.yaml"

# --------- Shared elements ----------
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
img_path = None
gt_dir = os.path.join(project_root, "data", "GT")
gt_paths = glob.glob(os.path.join(gt_dir, "*.png"))
saved_calib_images__dir_path = None
timestamp = None
ply_id = None
nc = None # counts ext calib frames
encoder = msgspec.json.Encoder() # pose encoder for serialization
mqtt_activation_event = threading.Event()
mqtt_calibration_event = threading.Event()
mqtt_termination_event = threading.Event()
mqtt_detect_event = threading.Event()
ext_calib_event = threading.Event()
options = apriltag.DetectorOptions(
    families="tag36h11",
    quad_decimate=1.0, # [Downsampling] (1.0) full-res quad detection
    quad_blur=1.0, # [Pre-smoothing] (1.0) Noisy / textured scenes
    refine_decode=True, # [Bit-decoding-refinement] helps borderline decodes
    refine_pose=False, # Irrelevant if custom solve PnP  
    refine_edges=True, # Improves corner/quad edge alignment to image gradients
    #quad_contours=True # robustness/alternative detection strategy option
)
detector = apriltag.Detector(options)
R_tcp2base = []   # each: (3,3)
t_tcp2base = []   # each: (3,1)
R_target2cam   = []   # each: (3,3)
t_target2cam   = []   # each: (3,1)
x_min, x_max = 390, 2190 # x limits for cropping due to lens vignetting
y_min, y_max = 290, 1790 # y limits for cropping due to lens vignetting
table_tag_size = 0.04822 # chosen table apriltag size in meters 
table_tag_ids = [77, 23, 101, 99] # chosen table apriltags
calib_tag_size = 0.16025 # calibration apriltag size in meters 
calib_tag_id = 101 # Apriltag ID used for calibration
roi_width = 0.866 # Distance between the two AprilTags centers along table width in meters
roi_length = 1.059 # Distance between the two AprilTags centers along table length in meters
gt_metric_length = 7.2 # Length of the y-axis of the GT in meters 
gt_pixel_length = cv2.imread(gt_paths[0], cv2.IMREAD_GRAYSCALE).shape[0] # Length of the y-axis of the GT in pixels 
# [48.22 mm] small one
# [96.30 mm] mid one
# [160.33 mm] big one
# PICKING POSE DEFINITION WRT GT [da rivedere]
# --- starting pose in matched_prealigned_gt ---
p_gt = np.array([1417, 2834], dtype=np.float32)
v_gt = np.array([np.sin(-np.pi/4), np.cos(-np.pi/4)], dtype=np.float32)  # unit dir (wrt +y)

# --------- typed struct to encode 6D poses ----------
class PlyPose9D(msgspec.Struct):
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float
    dx: float
    dy:float
    pi: float

# --------- typed struct to encode the termination ----------
class termination(msgspec.Struct):
    termination_id: str

# --------- MQTT callbacks ----------
def on_connect(client, userdata, flags, reason_code, properties):
    print("[MQTT] Connected, reason:", reason_code)
    print("\n")
    # Subscribe to the topics
    client.subscribe(TOPIC_TERMINATION, qos=MQTT_QOS)
    client.subscribe(TOPIC_DETECT, qos=MQTT_QOS)
    client.subscribe(TOPIC_TCP_TRANS, qos=MQTT_QOS)
    client.subscribe(TOPIC_EST_EXT, qos=MQTT_QOS)
    client.subscribe(TOPIC_CALIBRATION, qos=MQTT_QOS)

def on_message(client, userdata, msg):
    if msg.topic == TOPIC_TERMINATION:
        print("\n[MQTT] Termination message received.\n")
        mqtt_termination_event.set()

    if msg.topic == TOPIC_DETECT:
        global ply_id
        global img_path
        ply_id = msg.payload.decode("utf-8")
        print("\n[MQTT] Detection trigger received for: " + ply_id + "\n")
        img_path = os.path.join(cam_dir, f"{ply_id}.png")
        mqtt_activation_event.set()

    if msg.topic == TOPIC_TCP_TRANS:
        print("\n[MQTT] Robot transformation received:")
        # ---- Decode JSON ----
        payload = msg.payload.decode("utf-8")
        data = json.loads(payload)
        # ---- Rotation matrix (3x3) ----
        R = np.array(data["R"], dtype=np.float64)   # shape (3,3)
        assert R.shape == (3, 3)
        # ---- Translation (3x1) ----
        t = np.array(data["t"], dtype=np.float64).reshape(3, 1)
        # ---- Print decoded transform ----
        np.set_printoptions(precision=6, suppress=True)
        print("R_tcp2base:")
        print(R)
        print("t_tcp2base:")
        print(t.reshape(1, 3))
        # ---- Append for hand-eye ----
        R_tcp2base.append(R)
        t_tcp2base.append(t)
        # (Optional sanity check: R must be a valid rotation)
        if not np.allclose(R @ R.T, np.eye(3), atol=1e-6):
            print("[WARN] R is not orthonormal")
        mqtt_calibration_event.set()
    if msg.topic == TOPIC_EST_EXT:
        print("Estimating Extrinsics ...")
        # estimate cam-tcp transformation, save it to path
        estimate_extrinsics(ecalib_path)
    if msg.topic == TOPIC_CALIBRATION:
        global timestamp 
        global saved_calib_images__dir_path 
        global nc
        R_tcp2base.clear()
        t_tcp2base.clear()
        R_target2cam.clear()
        t_target2cam.clear()
        nc = 0
        print("\n[MQTT] Calibration Trigger received.\n")
        print("\n[MQTT] Starting Extrinsics Calibration ...\n")
        # build directory path for 
        timestamp = datetime.now().strftime("%H_%M_%S__%d%m%Y")
        saved_calib_images__dir_path = os.path.join(
            project_root,
            "calib/extrinsics",
            f"extrinsic_calibration_{timestamp}_images"
        )
        # create directory 
        os.makedirs(saved_calib_images__dir_path, exist_ok=True)


# --------- Create MQTT client ----------
def create_mqtt_client():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=MQTT_CLIENT_ID)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    client.loop_start()  # run network loop in background thread
    return client

# --------- Load calibration ----------
def load_calibration(icalib_path, ecalib_path):
    # --- Intrinsics ---
    fs = cv2.FileStorage(icalib_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open intrinsics calibration file: {icalib_path}")

    camera_matrix = fs.getNode("camera_matrix").mat()
    dist_coeffs = fs.getNode("distortion_coefficients").mat()
    fs.release()

    # --- Extrinsics: TCP <- Camera ---
    fs = cv2.FileStorage(ecalib_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open extrinsics calibration file: {ecalib_path}")

    R_tcp_cam = fs.getNode("R").mat()
    t_tcp_cam = fs.getNode("t").mat()
    fs.release()

    return camera_matrix, dist_coeffs, R_tcp_cam, t_tcp_cam


# --------- AprilTag detection ----------
def detect_apriltags(gray_img):
    tags = detector.detect(gray_img)
    return tags

# --------- Publish picking pose via MQTT ----------
def publish_picking_pose(client, rvec, tvec, dx, dy, theta):
    pose = PlyPose9D(
        x=float(tvec[0].item()),
        y=float(tvec[1].item()),
        z=float(tvec[2].item()),
        rx=float(rvec[0].item()),
        ry=float(rvec[1].item()),
        rz=float(rvec[2].item()),
        dx=float(dx),
        dy=float(dy),
        pi=float(theta),
    )
    payload = encoder.encode(pose)  # bytes
    result = client.publish(TOPIC_PICKING_POSE, payload=payload, qos=MQTT_QOS)
    rc,mid = result
    if rc == mqtt.MQTT_ERR_SUCCESS:
        print(f"[MQTT] Published picking pose successfully.\n")
    else:
        print(f"[MQTT] Failed to publish picking pose. Error code: {rc}\n")

# --------- Publish extrinsics calibration step done via MQTT ----------
def publish_calib_done_vision(client):
    result = client.publish(TOPIC_CALIB_DONE_VISION, payload="", qos=MQTT_QOS)
    rc,mid = result
    if rc == mqtt.MQTT_ERR_SUCCESS:
        print(f"\n[MQTT] Published calibration vision done message successfully.\n")
    else:
        print(f"\n[MQTT] Failed to publish the calibration vision done message. Error code: {rc}\n")

# --------- Calibration Step ----------
def calibration_step(
    latest_frame,
    K,
    dist,
    mqtt_client=None,
):
    global nc
    # Load image
    img = latest_frame
    disp = cv2.cvtColor(latest_frame, cv2.COLOR_GRAY2BGR)

    # AprilTags
    tags = detect_apriltags(img)
    n = len(tags)
    print(f"\nDetected AprilTags: {n}\n")

    found_calib = False

    # If multiple tags detected in the same frame, only process the calibration tag
    for t in tags:
        tag_id = t.tag_id
        if tag_id == calib_tag_id:
            found_calib = True
            center = t.center
            corners = t.corners

            corners_int = corners.astype(int)
            for i in range(4):
                p1 = tuple(corners_int[i])
                p2 = tuple(corners_int[(i + 1) % 4])
                cv2.line(disp, p1, p2, (0, 255, 0), 2)

            cv2.circle(disp, tuple(center.astype(int)), 3, (0, 0, 255), -1)
            cv2.putText(disp, str(tag_id), tuple(center.astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

            # solvePnP (tag->cam)
            half = calib_tag_size / 2.0
            objp = np.array(
                [
                    [-half,  half, 0],
                    [ half,  half, 0],
                    [ half, -half, 0],
                    [-half, -half, 0],
                ],
                dtype=np.float32,
            )
            imgp = corners.astype(np.float32)

            success, rvec, tvec = cv2.solvePnP(objp, imgp, K, dist)
            if success:
                R, _ = cv2.Rodrigues(rvec)                 # (3,3)
                t_cam = tvec.reshape(3, 1).astype(np.float64)  # (3,1)

                R_target2cam.append(R.astype(np.float64))
                t_target2cam.append(t_cam)

                print(f"  [Pose] wrt camera:\nR_tag=\n{R}\nt_tag=\n{t_cam.ravel()}\n")
            else:
                print("[ERROR] Pose Estimation Failed.\n")
            break # stop after first match
    
    # No calib tag detected: remove the last gripper pose that was appended for this frame.
    if not found_calib:
        # remove the gripper pose appended for this frame
        if R_tcp2base:
            R_tcp2base.pop()
        if t_tcp2base:
            t_tcp2base.pop()

    # Reset activation event after the pipeline
    mqtt_calibration_event.clear()
    return disp

# --------- Final Estimation ----------
def estimate_extrinsics(ecalib_path = "T_cam2tcp.yaml"):
    n_tb = len(R_tcp2base)
    n_tc = len(R_target2cam)

    print(f"\n[Hand–Eye] Samples:")
    print(f"  tcp→base: {n_tb}")
    print(f"  target→camera: {n_tc}\n")

    if n_tb == 0 or n_tc == 0:
        print("[ERROR] One or more input lists are empty")
        return None, None

    if n_tb != n_tc:
        print("[ERROR] List length mismatch")
        return None, None

    try:
        # (for fixed camera, pose inversion trick or eye-to-hand)
        # for camera attached to robot, eye-in-hand
        R_cam2tcp, t_cam2tcp = cv2.calibrateHandEye(
            R_tcp2base,
            t_tcp2base,
            R_target2cam,
            t_target2cam,
            method=cv2.CALIB_HAND_EYE_TSAI
        )
    except cv2.error as e:
        print("[WARN] calibrateHandEye failed:", e)
        R_cam2tcp = np.eye(3)
        t_cam2tcp = np.zeros((3, 1))

    t_cam2tcp = t_cam2tcp.reshape(3, 1)

    print("[Hand–Eye] Camera → TCP")
    print("R_cam2tcp =\n", R_cam2tcp)
    print("t_cam2tcp =", t_cam2tcp.ravel())

    save_ecalib_path = os.path.join(project_root, "calib/extrinsics", f"T_cam2tcp_{timestamp}.yaml")
    save_camera_tcp_yaml(
        save_ecalib_path,
        R_cam2tcp,
        t_cam2tcp
    )



# --------- Saving Extrinsics for re-usal ----------
def save_camera_tcp_yaml(path, R_cam2tcp, t_cam2tcp):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    fs.write("R", R_cam2tcp)
    fs.write("t", t_cam2tcp)
    fs.release()

def keep_big_edges_from_canny(
    edges_u8: np.ndarray,
    close_ksize: int = 5,
    close_iters: int = 2,
    dilate_iters: int = 1,
    keep: str = "largest",          # "largest" or "area_thresh"
    min_area: int = 3000,           # used if keep="area_thresh"
    thin: bool = False,
):
    """
    edges_u8: result of cv2.Canny (0/255)
    returns : cleaned edges (0/255)
    """

    e = (edges_u8 > 0).astype(np.uint8) * 255

    # 1) connect fragments
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_ksize, close_ksize))
    e = cv2.morphologyEx(e, cv2.MORPH_CLOSE, k, iterations=close_iters)
    if dilate_iters > 0:
        e = cv2.dilate(e, k, iterations=dilate_iters)

    # 2) connected components filtering
    num, labels, stats, _ = cv2.connectedComponentsWithStats(e, connectivity=8)

    out = np.zeros_like(e)
    if num <= 1:
        return out

    if keep == "largest":
        largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
        out[labels == largest] = 255
    else:
        # keep all components above area threshold
        for lab in range(1, num):
            if stats[lab, cv2.CC_STAT_AREA] >= min_area:
                out[labels == lab] = 255

    # 3) optional thinning back to 1px
    if thin:
        if hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "thinning"):
            out = cv2.ximgproc.thinning(out)
        else:
            # fallback: light erosion (not true thinning, but reduces thickness)
            out = cv2.erode(out, np.ones((3, 3), np.uint8), iterations=1)

    return out

# --------- Segment the table ROI ----------
def roiSegmentation(gray, roi):

    ns = 0
    
    # ----------------------- PRE-PROCESSING ---------------------------

    # ---- 1) Flat-field illumination correction (edge-preserving) ----
    g = gray.astype(np.float32) / 255.0
    # Estimate illumination (kernel dim relative to reflection/shading scale)
    illum = cv2.GaussianBlur(g, (0, 0), sigmaX=125, sigmaY=125)
    # Avoid division by tiny values
    eps = 1e-4
    corrected = g / (illum + eps)
    # Re-normalize to [0,1]
    corrected = cv2.normalize(corrected, None, 0.0, 1.0, cv2.NORM_MINMAX)
    corrected_u8 = (corrected * 255.0).astype(np.uint8)

    path = os.path.join(
        project_root, "data/saved_images", 
        f"Image_{ns}_FLAT_FIELD_ILLUM_CORRECTION_session_{session_timestamp}.png"
    )
    cv2.imwrite(path, corrected_u8)
    ns += 1

    # ---- 2) Edge-preserving denoise ----
    c_denoised = cv2.bilateralFilter(corrected_u8, d=7, sigmaColor=60, sigmaSpace=60)

    path = os.path.join(
        project_root, "data/saved_images", 
        f"Image_{ns}_BILATERAL_session_{session_timestamp}.png"
    )
    cv2.imwrite(path, c_denoised)
    ns += 1


    # ----------------------- SEGMENTATION PIPELINE ---------------------------

    rows, cols = gray.shape[:2]
    roi_mask = np.zeros((rows, cols), dtype=np.uint8) 
    cv2.fillConvexPoly(roi_mask, roi, 255)

    # 3) slight blur to make Otsu more stable (Reduce intra-class variance in the histogram)
    blur = cv2.GaussianBlur(c_denoised, (5, 5), 0)

    # 4) Otsu threshold (binary)
    roi_pixels = blur[roi_mask > 0] # compute th only using pixels inside of ROI
    T, _ = cv2.threshold(roi_pixels, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    # apply threshold to the *unblurred* image for sharper edges
    otsu = (c_denoised >= T).astype(np.uint8) * 255

    # 5) Mask edges to keep only inside ROI convex polygon 
    otsu_roi = cv2.bitwise_and(otsu, otsu, mask=roi_mask)

    path = os.path.join(
        project_root, "data/saved_images", 
        f"Image_{ns}_OTSU_{session_timestamp}.png"
    )
    cv2.imwrite(path, otsu_roi)
    ns += 1

    # 6) Erosion and Dilation to bridge-kill
    bkKernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    bridge_kill = cv2.erode(otsu_roi, bkKernel, iterations=5)      # breaks thin links

    path = os.path.join(
        project_root, "data/saved_images", 
        f"Image_{ns}_BRIDGE_KILL_{session_timestamp}.png"
    )
    cv2.imwrite(path, bridge_kill)
    ns += 1

    # 7) Largest Connected Component
    num, labels, stats, _ = cv2.connectedComponentsWithStats(bridge_kill, connectivity=8)
    # stats[0] is background. If there is at least 1 foreground component:
    if num > 1:
        largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])  # label id of biggest blob
        biggest_conn_comp = (labels == largest).astype(np.uint8) * 255
    else:
        biggest_conn_comp = np.zeros_like(otsu_roi)

    path = os.path.join(
        project_root, "data/saved_images", 
        f"Image_{ns}_CONNECT_COMP_{session_timestamp}.png"
    )
    cv2.imwrite(path, biggest_conn_comp)
    ns += 1

    # # 8) Border details with morphology
    details_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    morph = cv2.morphologyEx(biggest_conn_comp,    cv2.MORPH_CLOSE, details_kernel, iterations=20)

    path = os.path.join(
        project_root, "data/saved_images", 
        f"Image_{ns}_BORDER_DETAILS_{session_timestamp}.png"
    )
    cv2.imwrite(path, morph)
    ns += 1

    # 9) Fill up holes 
    ply_mask = morph.copy()
    h, w = ply_mask.shape[:2]
    ff = ply_mask.copy()
    ff_mask = np.zeros((h+2, w+2), np.uint8)
    # flood fill the background from (0,0)
    cv2.floodFill(ff, ff_mask, (0,0), 255)
    # holes are the background regions not connected to border
    holes = cv2.bitwise_not(ff)  # holes=255, else 0
    segmented_ply = cv2.bitwise_or(ply_mask, holes)

    return segmented_ply

# --------- Extract edges from convex-filled-shape ----------
def plyEdgeExtraction(refined_mask: np.ndarray, kernel_size: int = 3) -> np.ndarray:
    """
    Best-practice 1px edge extraction for Chamfer/DT:
    - clean mask lightly
    - extract external contour
    - draw contour with thickness=1

    Input:
      refined_mask: uint8 binary mask (0 background, 255 foreground)
    Output:
      edges: uint8 edge image (0 background, 255 edges), 1-pixel wide
    """
    if refined_mask.ndim != 2:
        raise ValueError("refined_mask must be a 2D (H,W) binary image")

    mask = (refined_mask > 0).astype(np.uint8) * 255

    # light cleanup (optional but usually helpful)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)

    # contour -> exact 1px edges
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    edges = np.zeros_like(mask)
    cv2.drawContours(edges, contours, -1, 255, thickness=1)

    return edges


# --------- PCA-based Alignment ----------
def symChamfer(aligned_gt_u8: np.ndarray, query_bin: np.ndarray, DT_query: np.ndarray) -> float:
        aligned = (aligned_gt_u8 > 0).astype(np.uint8) * 255
        query_bin = (query_bin > 0).astype(np.uint8) * 255
        if aligned.max() == 0 or query_bin.max() == 0:
            return float("inf")
        DT_gt = cv2.distanceTransform(255 - aligned, cv2.DIST_L2, 3)

        q_mask  = (query_bin > 0)
        gt_mask = (aligned > 0)
        if not np.any(q_mask) or not np.any(gt_mask):
            return float("inf")

        return float(np.mean(DT_gt[q_mask]) + np.mean(DT_query[gt_mask]))

def posePCA(img_u8: np.ndarray, min_pts: int = 10, ellipse_scale: float = 1.0,):
        ys, xs = np.where(img_u8 > 0)
        if len(xs) < min_pts:
            raise ValueError("Not enough edge pixels to compute PCA pose.")
        pts = np.stack([xs.astype(np.float32), ys.astype(np.float32)], axis=1)  # [x,y]
        c = pts.mean(axis=0).astype(np.float32)
        X = pts - c
        C = (X.T @ X) / float(len(pts))
        vals, vecs = np.linalg.eigh(C)
        order = np.argsort(vals)[::-1]
        vals = vals[order]
        vecs = vecs[:, order]
        v1 = vecs[:, 0].astype(np.float32)  # major axis
        r1 = float(np.sqrt(max(vals[0], 0.0)) * ellipse_scale)
        r2 = float(np.sqrt(max(vals[1], 0.0)) * ellipse_scale)
        return c, v1, (r1, r2)

def PCAlignment(
    query: np.ndarray,
    gt: np.ndarray,
    DT_query: np.ndarray,
    cq: Optional[np.ndarray] = None,
    vq: Optional[np.ndarray] = None,
    rq: Optional[Tuple[float, float]] = None,
    draw_debug: bool = False,
    min_pts: int = 10,
    ellipse_scale: float = 1.0,
):
    """
    Align `gt` to `query` using PCA pose (rotation + translation), selecting the best of the two
    PCA sign ambiguities via symmetric chamfer.

    If you pass (cq, vq, rq), query PCA is NOT recomputed.

    Inputs
      query, gt: 2D images (binary edges). Nonzero treated as foreground.
      DT_query: distance transform of query background (e.g. cv2.distanceTransform(255-query_bin,...))
      cq: (2,) query centroid in pixels [x,y]
      vq: (2,) query major axis unit vector (or any nonzero vector; will be normalized)
      rq: (r1,r2) query PCA radii (major, minor) in pixels (for debug ellipse only)
    Returns
      if not draw_debug: (gt_aligned_best, M_best, chamfer_best)
      else:             (gt_aligned_best, M_best, overlay, chamfer_best)
    """
    query_bin = (query > 0).astype(np.uint8) * 255
    gt_bin    = (gt > 0).astype(np.uint8) * 255
    H, W = query_bin.shape[:2]

    def _rotmat(theta: float) -> np.ndarray:
        c, s = float(np.cos(theta)), float(np.sin(theta))
        return np.array([[c, -s], [s, c]], dtype=np.float32)

    def _warp(img_u8: np.ndarray, M: np.ndarray) -> np.ndarray:
        return cv2.warpAffine(
            img_u8, M, (W, H),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0
        )

    # -------------------------
    # Query pose: use input if provided, else compute once here
    # -------------------------
    if cq is None or vq is None:
        cq2, vq2, rq2 = posePCA(query_bin)
        if cq is None: cq = cq2
        if vq is None: vq = vq2
        if rq is None: rq = rq2
    else:
        cq = np.asarray(cq, dtype=np.float32).reshape(2,)
        vq = np.asarray(vq, dtype=np.float32).reshape(2,)
        nv = float(np.linalg.norm(vq))
        if nv < 1e-9:
            raise ValueError("vq must be a nonzero vector.")
        vq = vq / nv
        if rq is None:
            rq = (1.0, 1.0)  # only used for debug ellipse

    # GT pose always computed (unless you also want to add cg/vg/rg inputs similarly)
    cg, vg, (rg1, rg2) = posePCA(gt_bin)

    aq = float(np.arctan2(vq[1], vq[0]))
    ag = float(np.arctan2(vg[1], vg[0]))

    # Two unique rotations due to PCA axis sign ambiguity
    deltas = [aq - ag, aq - (ag + np.pi)]

    best: Dict[str, object] = {"cost": float("inf"), "M": None, "gt_aligned": None, "aq": aq}
    for delta in deltas:
        R = _rotmat(delta)
        t = cq - (R @ cg)
        M = np.array([[R[0, 0], R[0, 1], t[0]],
                      [R[1, 0], R[1, 1], t[1]]], dtype=np.float32)

        gt_aligned = _warp(gt_bin, M)
        cost = symChamfer(gt_aligned, query_bin, DT_query)
        if cost < best["cost"]:
            best.update({"cost": cost, "M": M, "gt_aligned": gt_aligned})

    gt_aligned_best = best["gt_aligned"]
    M_best = best["M"]
    chamfer_best = best["cost"]

    if not draw_debug:
        return gt_aligned_best, M_best, chamfer_best

    # -------------------------
    # Debug overlay
    # -------------------------
    overlay = np.zeros((H, W, 3), dtype=np.uint8)
    overlay[:, :, 1] = (query_bin > 0).astype(np.uint8) * 255
    overlay[:, :, 2] = (gt_aligned_best > 0).astype(np.uint8) * 255

    cq_i = (int(round(float(cq[0]))), int(round(float(cq[1]))))
    cv2.circle(overlay, cq_i, 5, (0, 255, 0), -1)

    cg_h = np.array([cg[0], cg[1], 1.0], dtype=np.float32)
    cg_aligned = (M_best @ cg_h).astype(np.float32)
    cg_i = (int(round(float(cg_aligned[0]))), int(round(float(cg_aligned[1]))))
    cv2.circle(overlay, cg_i, 5, (0, 0, 255), -1)

    rq1, rq2 = rq
    q_axes = (max(1, int(round(float(rq1)))), max(1, int(round(float(rq2)))))
    g_axes = (max(1, int(round(float(rg1)))), max(1, int(round(float(rg2)))))

    cv2.ellipse(overlay, cq_i, q_axes, np.degrees(aq), 0, 360, (0, 255, 0), 2)
    cv2.ellipse(overlay, cg_i, g_axes, np.degrees(aq), 0, 360, (0, 0, 255), 2)

    return gt_aligned_best, M_best, overlay, chamfer_best


# --------- ORB-based Alignment ----------
def _enforce_rigid_affine(M: np.ndarray) -> np.ndarray:
    """
    Convert a 2x3 affine/similarity matrix into the closest rigid transform (R,t),
    removing any scale/shear via SVD projection onto SO(2).
    """
    A = M[:, :2].astype(np.float64)   # 2x2
    t = M[:, 2].astype(np.float64)    # 2,

    # Project A onto nearest rotation: A ~= U S Vt  ->  R = U Vt
    U, _, Vt = np.linalg.svd(A)
    R = U @ Vt
    # Ensure det(R)=+1 (proper rotation)
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt

    Mr = np.zeros((2, 3), dtype=np.float64)
    Mr[:, :2] = R
    Mr[:, 2] = t
    return Mr.astype(np.float32)

def ORBalignemnt(
    query: np.ndarray,
    gt: np.ndarray,
    max_corners: int = 12,
    dilate_iters: int = 1,
    orb_nfeatures: int = 50,
    keep_best_matches: int = 4,
    ransac_thresh_px: float = 0.75,
    harris_kernel_size: int = 9,
):
    q_bin = (query == 255).astype(np.uint8) * 255
    g_bin = (gt    == 255).astype(np.uint8) * 255

    if q_bin.ndim != 2 or g_bin.ndim != 2:
        raise ValueError("Input images must be single-channel binary.")
    H, W = q_bin.shape
    if g_bin.shape != (H, W):
        raise ValueError("Query and GT must have same shape.")

    # features
    if dilate_iters > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        q_feat = cv2.dilate(q_bin, k, iterations=dilate_iters)
        g_feat = cv2.dilate(g_bin, k, iterations=dilate_iters)
    else:
        q_feat, g_feat = q_bin.copy(), g_bin.copy()

    q_feat = cv2.GaussianBlur(q_feat, (5, 5), 0)
    g_feat = cv2.GaussianBlur(g_feat, (5, 5), 0)

    q_corners = cv2.goodFeaturesToTrack(
        q_feat,
        maxCorners=max_corners,
        qualityLevel=0.01,
        minDistance=10,
        blockSize=int(harris_kernel_size),
        useHarrisDetector=True,
        k=0.04
    )
    g_corners = cv2.goodFeaturesToTrack(
        g_feat,
        maxCorners=max_corners,
        qualityLevel=0.01,
        minDistance=10,
        blockSize=int(harris_kernel_size),
        useHarrisDetector=True,
        k=0.04
    )

    overlay_init = np.zeros((H, W, 3), np.uint8)
    overlay_init[..., 1] = q_bin
    overlay_init[..., 2] = g_bin

    if q_corners is None or g_corners is None:
        return overlay_init, None, overlay_init.copy(), float("inf"), None

    q_corners = q_corners.reshape(-1, 2).astype(np.float32)
    g_corners = g_corners.reshape(-1, 2).astype(np.float32)

    kp_q = [cv2.KeyPoint(float(x), float(y), 31.0) for (x, y) in q_corners]
    kp_g = [cv2.KeyPoint(float(x), float(y), 31.0) for (x, y) in g_corners]

    orb = cv2.ORB_create(
        nfeatures=orb_nfeatures,
        fastThreshold=5,
        edgeThreshold=5,
        patchSize=31
    )
    kp_q, des_q = orb.compute(q_feat, kp_q)
    kp_g, des_g = orb.compute(g_feat, kp_g)

    if des_q is None or des_g is None or len(kp_q) < 2 or len(kp_g) < 2:
        return overlay_init, None, overlay_init.copy(), float("inf"), None

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des_g, des_q)  # GT -> Query
    matches = sorted(matches, key=lambda m: m.distance)
    matches = matches[:min(keep_best_matches, len(matches))]

    for m in matches:
        xg, yg = kp_g[m.queryIdx].pt
        xq, yq = kp_q[m.trainIdx].pt

        pg = (int(round(xg)), int(round(yg)))
        pq = (int(round(xq)), int(round(yq)))

        cv2.line(overlay_init, pg, pq, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.circle(overlay_init, pg, 15, (0, 0, 255), -1)   # GT
        cv2.circle(overlay_init, pq, 15, (0, 255, 0), -1)   # Query

    src = np.float32([kp_g[m.queryIdx].pt for m in matches])
    dst = np.float32([kp_q[m.trainIdx].pt for m in matches])

    M, inliers = cv2.estimateAffinePartial2D(
        src,
        dst,
        method=cv2.RANSAC,
        ransacReprojThreshold=ransac_thresh_px,
        maxIters=5000,
        confidence=0.995,
        refineIters=20,
    )
    if M is None:
        return overlay_init, None, overlay_init.copy(), float("inf"), None

    M = _enforce_rigid_affine(M)  # removes scaling

    # GT transformed according to M
    gt_aligned = cv2.warpAffine(
        g_bin,
        M,
        (W, H),
        flags=cv2.INTER_NEAREST,
        borderValue=0
    )

    overlay_final = np.zeros((H, W, 3), np.uint8)
    overlay_final[..., 1] = q_bin
    overlay_final[..., 2] = gt_aligned

    DT_gt = cv2.distanceTransform(255 - gt_aligned, cv2.DIST_L2, 3).astype(np.float32)
    DT_q  = cv2.distanceTransform(255 - q_bin,      cv2.DIST_L2, 3).astype(np.float32)

    q_mask = (q_bin == 255)
    g_mask = (gt_aligned == 255)

    chamfer = float("inf")
    if np.any(q_mask) and np.any(g_mask):
        chamfer = float(np.mean(DT_gt[q_mask]) + np.mean(DT_q[g_mask]))

    return overlay_init, M, overlay_final, chamfer, gt_aligned

# --------- ICP Refinement ----------
def ICPrefinement(
    target_bin: np.ndarray,
    source_bin: np.ndarray,
    DT_query: np.ndarray,
    # Optimization
    max_iters: int = 300,
    tol: float = 1e-6,
    multiscale_steps=(6, 3, 1),
    # Robustness / partial handORBalignemntling
    trim_fraction: float = 0.25,     # drop worst residuals each iter (applied per-term)
    huber_delta: float = 2.5,        # px (scaled by stride)
    gate_max_dist: float = 20.0,     # px (scaled by stride) hard-gate DT residuals
    # Symmetric term (prevents "sliding" along a single aligned edge)
    sym_weight: float = 1.0,         # weight for target->source term
    # Coverage penalties (break degeneracy further)
    use_coverage_penalty: bool = True,
    cover_eps: float = 2.0,          # px (scaled by stride)
    cover_weight: float = 0.25,      # penalty weight for (1-coverage)
    # Step control
    gn_damp: float = 1e-6,           # LM damping
    max_rot_step_deg: float = 5.0,   # clamp per-iter rotation update
    max_trans_step_px: float = 10.0, # clamp per-iter translation update
    # Init
    init_R: np.ndarray | None = None,
    init_t: np.ndarray | None = None,
    allow_translation: bool = True,
    # Output
    return_overlay: bool = True,
):
    """
    Symmetric distance-transform Gauss-Newton refinement for 1px binary edge maps.

    Minimizes (robustly):
      E = mean( rho( DT_tgt( R x + t ) ) )  + sym_weight * mean( rho( DT_src( R^T (y - t) ) ) )
          + cover_weight * [(1-cov_s2t) + sym_weight*(1-cov_t2s)]   (optional)

    This symmetric term removes the common degeneracy where a partial straight edge can slide along
    a long edge of the full shape with near-zero DT cost.

    Returns: R (2,2), t (2,), overlay_rgb (H,W,3) uint8 or None, chamfer_cost (float)
    """
    target = (target_bin.astype(np.uint8) > 0).astype(np.uint8)
    source = (source_bin.astype(np.uint8) > 0).astype(np.uint8)
    if target.shape != source.shape:
        raise ValueError(f"Shape mismatch: target{target.shape} vs source{source.shape}")
    H, W = target.shape

    def edge_points(img_u8):
        ys, xs = np.nonzero(img_u8)
        return np.stack([xs, ys], axis=1).astype(np.float64)

    tgt_pts_full = edge_points(target)
    src_pts_full = edge_points(source)
    if len(tgt_pts_full) < 10 or len(src_pts_full) < 10:
        raise ValueError("Not enough edge pixels in one of the images.")

    def subsample_stride(pts, stride):
        if stride <= 1:
            return pts
        q = np.floor(pts / stride).astype(np.int64)
        _, idx = np.unique(q, axis=0, return_index=True)
        return pts[np.sort(idx)]

    def dist_transform_to_edges(edge_u8):
        inv = np.ones_like(edge_u8, dtype=np.uint8)
        inv[edge_u8 > 0] = 0
        return cv2.distanceTransform(inv, distanceType=cv2.DIST_L2, maskSize=3)

    def sample_bilinear(img, x, y):
        x0 = np.floor(x).astype(np.int32)
        y0 = np.floor(y).astype(np.int32)
        x1 = x0 + 1
        y1 = y0 + 1

        inb = (x0 >= 0) & (x1 < W) & (y0 >= 0) & (y1 < H)
        out = np.zeros_like(x, dtype=np.float64)
        if not np.any(inb):
            return out, inb

        xa = x[inb] - x0[inb]
        ya = y[inb] - y0[inb]

        Ia = img[y0[inb], x0[inb]]
        Ib = img[y0[inb], x1[inb]]
        Ic = img[y1[inb], x0[inb]]
        Id = img[y1[inb], x1[inb]]

        out[inb] = (1 - xa) * (1 - ya) * Ia + xa * (1 - ya) * Ib + (1 - xa) * ya * Ic + xa * ya * Id
        return out, inb

    def huber_rho_and_w(r, delta):
        r = np.asarray(r, dtype=np.float64)
        rho = np.empty_like(r)
        w = np.ones_like(r)

        m = r > delta
        rho[~m] = 0.5 * r[~m] ** 2
        rho[m] = delta * (r[m] - 0.5 * delta)

        w[m] = delta / (r[m] + 1e-12)  # IRLS weight
        return rho, w

    def clamp_step(dtheta, dt, max_rot_deg, max_trans_px):
        max_rot = np.deg2rad(max_rot_deg)
        dtheta = float(np.clip(dtheta, -max_rot, max_rot))
        dt = np.asarray(dt, dtype=np.float64)
        n = float(np.linalg.norm(dt))
        if n > max_trans_px:
            dt = dt * (max_trans_px / (n + 1e-12))
        return dtheta, dt

    # ---- Paramization: theta + t (easier symmetric Jacobians than left-multiplicative updates) ----
    if init_R is None:
        theta = 0.0
    else:
        R0 = np.asarray(init_R, dtype=np.float64)
        theta = float(np.arctan2(R0[1, 0], R0[0, 0]))  # assumes proper rotation

    t = np.zeros(2, dtype=np.float64) if init_t is None else np.asarray(init_t, dtype=np.float64).copy()

    def R_of(theta_):
        c = np.cos(theta_)
        s = np.sin(theta_)
        return np.array([[c, -s],
                         [s,  c]], dtype=np.float64)

    # Precompute DT + gradients for both images (static in their own coordinate frames)
    dt_tgt = dist_transform_to_edges(target).astype(np.float32)
    dt_src = dist_transform_to_edges(source).astype(np.float32)

    gx_tgt = cv2.Sobel(dt_tgt, cv2.CV_32F, 1, 0, ksize=3)
    gy_tgt = cv2.Sobel(dt_tgt, cv2.CV_32F, 0, 1, ksize=3)
    gx_src = cv2.Sobel(dt_src, cv2.CV_32F, 1, 0, ksize=3)
    gy_src = cv2.Sobel(dt_src, cv2.CV_32F, 0, 1, ksize=3)

    prev_obj = np.inf

    for stride in multiscale_steps:
        src_pts = subsample_stride(src_pts_full, stride)
        tgt_pts = subsample_stride(tgt_pts_full, stride)

        delta = huber_delta * max(1.0, float(stride))
        gate = gate_max_dist * max(1.0, float(stride))
        cover_thr = cover_eps * max(1.0, float(stride))

        for _ in range(max_iters):
            R = R_of(theta)
            c, s = float(np.cos(theta)), float(np.sin(theta))

            # dR/dtheta
            dR = np.array([[-s, -c],
                           [ c, -s]], dtype=np.float64)

            # -------------------------
            # Term A: source -> target
            # rA = DT_tgt( X ), X = R x + t
            # -------------------------
            X = (src_pts @ R.T) + t  # (Ns,2) in target coords
            xa, ya = X[:, 0], X[:, 1]

            rA, inbA = sample_bilinear(dt_tgt, xa, ya)
            gxA, inbAx = sample_bilinear(gx_tgt, xa, ya)
            gyA, inbAy = sample_bilinear(gy_tgt, xa, ya)
            inbA = inbA & inbAx & inbAy
            if not np.any(inbA):
                break

            X_in = X[inbA]
            src_in = src_pts[inbA]
            rA = rA[inbA]
            gxA = gxA[inbA]
            gyA = gyA[inbA]

            # gate + trim on rA
            if gate > 0:
                gkeep = rA < gate
                if np.count_nonzero(gkeep) < 10:
                    break
                X_in = X_in[gkeep]
                src_in = src_in[gkeep]
                rA = rA[gkeep]
                gxA = gxA[gkeep]
                gyA = gyA[gkeep]

            if trim_fraction > 0:
                keep_n = max(10, int((1.0 - trim_fraction) * len(rA)))
                kk = np.argpartition(rA, keep_n - 1)[:keep_n]
                X_in = X_in[kk]
                src_in = src_in[kk]
                rA = rA[kk]
                gxA = gxA[kk]
                gyA = gyA[kk]

            # normalize gradient to get stable normals
            gnA = np.sqrt(gxA * gxA + gyA * gyA) + 1e-12
            nxA = gxA / gnA
            nyA = gyA / gnA

            # dX/dtheta = dR * x
            dX_dth = src_in @ dR.T  # (M,2)
            Jth_A = nxA * dX_dth[:, 0] + nyA * dX_dth[:, 1]
            Jtx_A = nxA
            Jty_A = nyA

            rhoA, wA = huber_rho_and_w(rA, delta)

            covA = float(np.mean(rA < cover_thr)) if rA.size else 0.0
            cov_pen_A = (1.0 - covA) if use_coverage_penalty else 0.0

            # -------------------------
            # Term B: target -> source  (symmetric term)
            # rB = DT_src( P ), P = R^T (y - t)
            # -------------------------
            Y = tgt_pts  # (Nt,2) in target coords
            Yc = Y - t   # (Nt,2)
            P = Yc @ R   # (Nt,2) in source coords  (since R^T*(y-t) == (y-t)@R)
            xb, yb = P[:, 0], P[:, 1]

            rB, inbB = sample_bilinear(dt_src, xb, yb)
            gxB, inbBx = sample_bilinear(gx_src, xb, yb)
            gyB, inbBy = sample_bilinear(gy_src, xb, yb)
            inbB = inbB & inbBx & inbBy
            if not np.any(inbB):
                break

            Y_in = Y[inbB]
            Yc_in = Yc[inbB]
            P_in = P[inbB]
            rB = rB[inbB]
            gxB = gxB[inbB]
            gyB = gyB[inbB]

            if gate > 0:
                gkeep = rB < gate
                if np.count_nonzero(gkeep) < 10:
                    break
                Y_in = Y_in[gkeep]
                Yc_in = Yc_in[gkeep]
                P_in = P_in[gkeep]
                rB = rB[gkeep]
                gxB = gxB[gkeep]
                gyB = gyB[gkeep]

            if trim_fraction > 0:
                keep_n = max(10, int((1.0 - trim_fraction) * len(rB)))
                kk = np.argpartition(rB, keep_n - 1)[:keep_n]
                Y_in = Y_in[kk]
                Yc_in = Yc_in[kk]
                P_in = P_in[kk]
                rB = rB[kk]
                gxB = gxB[kk]
                gyB = gyB[kk]

            gnB = np.sqrt(gxB * gxB + gyB * gyB) + 1e-12
            nxB = gxB / gnB
            nyB = gyB / gnB

            # P = (Y - t) @ R
            # dP/dtheta = (Y - t) @ dR
            dP_dth = Yc_in @ dR
            Jth_B = nxB * dP_dth[:, 0] + nyB * dP_dth[:, 1]

            # dP/dt = -(I) @ R  => dp/dtx = -[c, s], dp/dty = -[-s, c]?? careful:
            # R = [[c,-s],[s,c]]
            # (Y - t)@R : derivative wrt tx is -(1,0)@R = -[c, -s]
            # derivative wrt ty is -(0,1)@R = -[s,  c]
            dp_dtx = np.array([ -c,  s], dtype=np.float64) * 0.0  # placeholder (not used)
            # We'll compute Jtx_B, Jty_B directly:
            # dp/dtx = -[c, -s], dp/dty = -[s,  c]
            Jtx_B = nxB * (-c) + nyB * (s)     # dot(n, [-c, s]) is NOT correct; should be dot(n, -[c,-s]) = -nx*c + ny*(-s)? Let's do explicit:
            # dp/dtx = (-c, +s?) No: -[c, -s] = (-c, +s) yes.
            # dp/dty = -[s,  c] = (-s, -c)
            Jty_B = nxB * (-s) + nyB * (-c)

            # Correct Jtx_B is dot(n, dp/dtx) = nx*(-c) + ny*(+s)
            # Correct Jty_B is dot(n, dp/dty) = nx*(-s) + ny*(-c)

            rhoB, wB = huber_rho_and_w(rB, delta)

            covB = float(np.mean(rB < cover_thr)) if rB.size else 0.0
            cov_pen_B = (1.0 - covB) if use_coverage_penalty else 0.0

            # -------------------------
            # Combine into one GN system on [dtheta, dtx, dty]
            # -------------------------
            # Weighted objective (Point 4): mean robust costs + coverage penalties
            obj = float(
                np.mean(rhoA)
                + sym_weight * np.mean(rhoB)
                + cover_weight * (cov_pen_A + sym_weight * cov_pen_B)
            )

            # Stack residuals and Jacobians
            JA = np.stack([Jth_A, Jtx_A, Jty_A], axis=1)
            JB = np.stack([Jth_B, Jtx_B, Jty_B], axis=1)

            r = np.concatenate([rA, np.sqrt(sym_weight) * rB], axis=0)
            J = np.concatenate([JA, np.sqrt(sym_weight) * JB], axis=0)
            w = np.concatenate([wA, wB], axis=0)  # weights already correspond to r magnitudes; sym handled by sqrt in r,J

            # IRLS normal equations
            JW = J * w[:, None]
            Hm = JW.T @ J + gn_damp * np.eye(3)
            bm = JW.T @ r

            try:
                dp = -np.linalg.solve(Hm, bm)
            except np.linalg.LinAlgError:
                break

            dtheta = float(dp[0])
            dt_step = dp[1:3]
            if not allow_translation:
                dt_step[:] = 0.0

            dtheta, dt_step = clamp_step(dtheta, dt_step, max_rot_step_deg, max_trans_step_px)

            theta += dtheta
            t += dt_step

            if np.isfinite(prev_obj) and abs(prev_obj - obj) < tol:
                prev_obj = obj
                break
            prev_obj = obj

    # Final R
    R = R_of(theta)

    # Build affine for warping source -> target frame
    M = np.array([[R[0, 0], R[0, 1], t[0]],
                [R[1, 0], R[1, 1], t[1]]], dtype=np.float32)

    # Warp source into target frame (aligned_gt_u8 in your function)
    aligned_src_u8 = cv2.warpAffine(
        (source > 0).astype(np.uint8) * 255,  # source binary -> 0/255
        M, (W, H),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0
    )

    # --- Symmetric chamfer cost (final evaluation) ---
    # _sym_chamfer returns (mean DT_aligned at query pixels) + (mean DT_query at aligned pixels)
    chamfer_cost = symChamfer(aligned_src_u8, target, DT_query)

    # --- Overlay ---
    overlay = None
    if return_overlay:
        warped = (aligned_src_u8 > 0).astype(np.uint8)
        overlay = np.zeros((H, W, 3), dtype=np.uint8)
        overlay[..., 2] = ((target > 0).astype(np.uint8) * 255)  # red (target)
        overlay[..., 1] = (warped * 255)                         # green (warped source)

    return R, t, overlay, chamfer_cost

# [UNUSED]
def estimateQueryScale(tl, tr, br, bl,
                                  width_m,
                                  length_m,
                                  robust=False):
    """
    Estimate unique meters-per-pixel scale from 4 table corners.

    Parameters
    ----------
    tl, tr, br, bl : array-like (2,)
        Table corner points in pixels (x, y).
    table_width_m : float
        Real table width in meters (corresponds to top & bottom edges).
    table_length_m : float
        Real table length in meters (corresponds to left & right edges).
    Returns
    -------
    scale_m_per_px : float
        Estimated meters per pixel.
    """

    tl = np.asarray(tl, dtype=np.float64)
    tr = np.asarray(tr, dtype=np.float64)
    br = np.asarray(br, dtype=np.float64)
    bl = np.asarray(bl, dtype=np.float64)

    # Edge lengths in pixels
    w_top_px = np.linalg.norm(tr - tl)
    w_bot_px = np.linalg.norm(br - bl)
    l_left_px = np.linalg.norm(bl - tl)
    l_right_px = np.linalg.norm(br - tr)

    eps = 1e-9
    if min(w_top_px, w_bot_px, l_left_px, l_right_px) < eps:
        raise ValueError("Degenerate edge detected (near-zero length).")

    # Per-edge scales (m/px)
    scales = np.array([
        width_m  / w_top_px,
        width_m  / w_bot_px,
        length_m / l_left_px,
        length_m / l_right_px
    ], dtype=np.float64)

    if robust:
        return float(np.median(scales))

    # Length-weighted mean
    weights = np.array([
        w_top_px,
        w_bot_px,
        l_left_px,
        l_right_px
    ], dtype=np.float64)

    return float(np.sum(weights * scales) / np.sum(weights))



def crop_gt_bottom_to_match_query_yspan(gt, query):
    """
    Crop the lower part of GT foreground so that its y-span matches QUERY's y-span.

    Assumptions:
    - gt and query have the same shape
    - both contain at least one non-black pixel
    - foreground = pixel > 0
    """

    q_mask = query > 0
    g_mask = gt > 0

    q_rows = np.where(np.any(q_mask, axis=1))[0]
    g_rows = np.where(np.any(g_mask, axis=1))[0]

    q_y0, q_y1 = q_rows[0], q_rows[-1]
    g_y0, g_y1 = g_rows[0], g_rows[-1]

    query_span = q_y1 - q_y0 + 1
    gt_span = g_y1 - g_y0 + 1

    # if GT is already shorter or equal, leave it unchanged
    if gt_span <= query_span:
        return gt.copy()

    # keep only the top part of GT foreground band
    new_g_y1 = g_y0 + query_span - 1

    gt_new = gt.copy()
    gt_new[new_g_y1 + 1:, :] = 0

    return gt_new


# # --------- Draw Table Ref on query ----------
def tableAprilTagOrientation(img_bgr, rvec, center=None, axis_len=100, thickness=3):
    """
    Draw y axis of the detected Apriltag in the table
    """
    h, w = img_bgr.shape[:2]
    if center is None:
        center = (w // 2, h // 2)
    # Convert rvec to rotation matrix
    R, _ = cv2.Rodrigues(rvec)
    # Object frame unit axes
    axes = np.eye(3, dtype=np.float32)
    # Rotate axes
    axes_rot = R @ axes
    # We visualize only X and Y components in image plane
    cx, cy = center
    out = img_bgr.copy()
    dx = axis_len * axes_rot[0, 1]
    dy = axis_len * axes_rot[1, 1]
    pt = (int(cx + dx), int(cy + dy))
    cv2.arrowedLine(out, center, pt, (0,255,0), thickness, tipLength=0.25)
    return out

# --------- Main pipeline ----------
def run_pipeline(
    latest_frame,
    K,
    dist,
    R_cam2tcp,
    t_cam2tcp,
    mqtt_client=None
):

    # undistort the image first
    path = os.path.join(
        project_root, "data/saved_images", 
        f"Image_START_{session_timestamp}.png"
    )
    cv2.imwrite(path, latest_frame)

    newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, latest_frame.shape[:2], alpha=0.0)
    latest_frame = cv2.undistort(latest_frame, K, dist, None, newK)

    path = os.path.join(
        project_root, "data/saved_images", 
        f"Image_UNDISTORT_{session_timestamp}.png"
    )
    cv2.imwrite(path, latest_frame)

    img = latest_frame
    disp = cv2.cvtColor(latest_frame, cv2.COLOR_GRAY2BGR)
    proc = None

    # AprilTags
    blur = cv2.GaussianBlur(img, (0,0), 1.0)
    sharp = cv2.addWeighted(img, 1.5, blur, -0.5, 0)
    img = np.clip(sharp, 0, 255).astype(np.uint8)
    tags = detect_apriltags(img)
    print("\nDetected AprilTags:")
    centers = []   # store table apriltag centers 
    imgps   = []   # store corresponding 4x2 image points (corners) for each center
    tag_ids = []   # store corresponding ids for each center
    shrink_px = 0.0
    table_width_pixels = 0.0
    for t in tags:
        tag_id = t.tag_id
        center = t.center.astype(np.float32)
        corners = t.corners.astype(np.float32)
        print(f"[{tag_id}]")

        # save the table tags centers + corresponding imgp (corners)
        if tag_id in table_tag_ids:
            centers.append(center)
            imgps.append(corners.astype(np.float32))  # 4x2
            tag_ids.append(tag_id)

        # --- compute shrink_px ---
        dists = np.linalg.norm(corners - center[None, :], axis=1)
        shrink_px = max(shrink_px, dists.max())

        corners_int = corners.astype(int)
        for i in range(4):
            p1 = tuple(corners_int[i])
            p2 = tuple(corners_int[(i + 1) % 4])
            cv2.line(disp, p1, p2, (0, 255, 0), 2)

        # draw center
        cv2.circle(disp, tuple(center.astype(int)), 3, (0, 0, 255), -1)
        cv2.putText(
            disp, str(tag_id), tuple(center.astype(int)),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2
        )

        display_image(disp)
        pump_gui_for(0.3)

    # -------------- Table ROI -----------------
    if not len(centers) == 4:  
        print("\n[ERROR]: Not all four Table Apriltags were detected! \n")
        # Reset activation flag after the pipeline
        mqtt_activation_event.clear()
        return None,None

    print("\nTable fully detected!")
    pts = np.asarray(centers, dtype=np.float32).reshape(-1, 2)

    # Order points consistently: [top-left, top-right, bottom-right, bottom-left]
    s = pts.sum(axis=1)         # x + y
    d = (pts[:, 0] - pts[:, 1]) # x - y

    tl_idx = int(np.argmin(s))
    br_idx = int(np.argmax(s))
    tr_idx = int(np.argmax(d))
    bl_idx = int(np.argmin(d))

    tl = pts[tl_idx]
    br = pts[br_idx]
    tr = pts[tr_idx]
    bl = pts[bl_idx]

    # --- pick the imgp corresponding to TL center and solvePnP once ---
    imgp_tl = imgps[tl_idx].astype(np.float32)  # 4x2 corners for TL tag
    print(f"Table reference corresponding id: [{tag_ids[tl_idx]}]\n")

    # Tag is a square table_tag_size x table_tag_size in 3D
    half = table_tag_size / 2.0
    objp = np.array(
        [
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ],
        dtype=np.float32,
    )

    # [Image is undistorted already!] Use new intrinsic matrix.
    success, rvec, tvec = cv2.solvePnP(
        objp, imgp_tl, newK, None, flags=cv2.SOLVEPNP_IPPE_SQUARE
    )

    if not success:
        print("[ERROR] solvePnP failed: Table Pose could not be estimated.")
        # Reset activation flag after the pipeline
        mqtt_activation_event.clear()
        return None, None
        
    # -------------- create ROI -----------------

    init_roi = np.asarray([tl, tr, br, bl], dtype=np.int32)
    # Shrink initial ROI 
    c = init_roi.mean(axis=0)
    v = init_roi - c
    n = np.linalg.norm(v, axis=1, keepdims=True)
    n[n < 1e-6] = 1.0
    roi = init_roi - shrink_px * (v / n)
    roi = roi.astype(np.int32).reshape(-1, 1, 2)
    pts = roi.reshape(-1, 2)  # (4,2)
    bottom = pts[np.argsort(pts[:, 1])[-2:]]      # two largest y for the bottom
    b1_shrunk = tuple(bottom[0])  # (x,y)
    b2_shrunk = tuple(bottom[1])  # (x,y)
    # Draw ROI's on image 
    cv2.polylines(
        disp,
        [roi],
        isClosed=True,
        color=(0, 255, 255),
        thickness=2
    )

    display_image(disp)
    pump_gui_for(0.2)

    path = os.path.join(
        project_root, "data/saved_images", 
        f"Image_TABLE_DETECTION_{session_timestamp}.png"
    )
    cv2.imwrite(path, disp)
    

    # ---------------------------------------------------
    # --------------- ROI Segmentation ------------------
    # ---------------------------------------------------

    initial_candidate = roiSegmentation(img, roi)

    display_image(initial_candidate)
    pump_gui_for(0.2)

    segmented_ply = initial_candidate

    # ---------------------------------------------------
    # -------------- PLY SHAPE MATCHING -----------------
    # ---------------------------------------------------

    # -------------------------
    # 1) Query metric scale
    # -------------------------
    # Edge lengths in pixels
    w_top_px = np.linalg.norm(tr - tl)
    scale_query = roi_width/w_top_px

    # -------------------------
    # 2) GT metric scale 
    # -------------------------
    # using y direction (vertical)
    scale_gt = gt_metric_length / gt_pixel_length

    # -------------------------
    # 3) Compute resize factor so query pixels become comparable to GT pixels
    # -------------------------
    f = scale_query / scale_gt

    # -------------------------
    # 4) Rescale query
    # -------------------------
    h, w = segmented_ply.shape[:2]
    rescaled_w = max(1, int(round(w * f)))
    rescaled_h = max(1, int(round(h * f)))
    segmented_ply_rescaled = cv2.resize(
        segmented_ply,
        (rescaled_w, rescaled_h),
        interpolation=cv2.INTER_NEAREST  # preserve {0,255}
    )
    segmented_ply_rescaled = (segmented_ply_rescaled > 0).astype(np.uint8) * 255
    ply_edges_rescaled = segmented_ply_rescaled

    # extract edges from segmentation
    ply_edges_rescaled = plyEdgeExtraction(segmented_ply_rescaled)
    # skeletonize to 1px-wide
    ply_edges_rescaled = skeletonize(ply_edges_rescaled, method= "lee")

    query = ply_edges_rescaled
    query_h, query_w = query.shape[:2]
    query = (query > 0).astype(np.uint8) * 255

    # remove bottom edge if the ply is partial view
    # [NOTE]: to make the "fake" edge removal 
    # camera-rotational invariant, draw the same 
    # line using the other corners, for all of them
    # rescale points
    b1_rescaled = np.array(b1_shrunk, dtype=np.float32) * f
    b2_rescaled = np.array(b2_shrunk, dtype=np.float32) * f
    b1_pt = (int(round(b1_rescaled[0])), int(round(b1_rescaled[1])))
    b2_pt = (int(round(b2_rescaled[0])), int(round(b2_rescaled[1])))
    # draw the two points (use gray so visible on binary image)
    # cv2.circle(query, b1_pt, radius=5, color=128, thickness=-1)
    # cv2.circle(query, b2_pt, radius=5, color=128, thickness=-1)
    # then draw the line
    cv2.line(
        query,
        pt1=b1_pt,
        pt2=b2_pt,
        color=0,
        thickness=100
    )

    # Largest Connected Component, to avoid final outlier edges
    num, labels, stats, _ = cv2.connectedComponentsWithStats(query, connectivity=8)
    # stats[0] is background. If there is at least 1 foreground component:
    if num > 1:
        largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])  # label id of biggest blob
        query = (labels == largest).astype(np.uint8) * 255
    else:
        query = np.zeros_like(query)

    query_save = display_shapes(query, w, h, 0.2)
    if query_save is not None:
            save_path = os.path.join(
                project_root, "data/saved_images", 
                f"Image_QUERY_session_{session_timestamp}.png"
            )
            cv2.imwrite(save_path, query_save)

    # -------------------------
    # PCA-CHAMFER MATCHING LOOP
    # -------------------------

    # Distance transform of query (for symmetric term)
    DT_Query = cv2.distanceTransform(255 - query, cv2.DIST_L2, 3)
    cq, vq, rq = posePCA(query)
    
    best_score = float("inf")
    result = None
    matched_id = None
    matched_gt = None
    matched_prealigned_gt = None
    gt_h = None
    gt_w = None
    T = None
    print("[PCA-Matching]")
    print("Listing per-ply symmetric-Chamfer costs:")

    for path in gt_paths:

        # --- Load GT ---
        gt = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        gt = (gt > 0).astype(np.uint8) * 255

        # --- Make same size as query (top-left aligned) ---
        gt_resized = np.zeros_like(query)  # query is (rescaled_h, rescaled_w)
        h_min = min(query_h, gt.shape[0])
        w_min = min(query_w, gt.shape[1])
        gt_resized[:h_min, :w_min] = \
            gt[:h_min, :w_min]
        gt = gt_resized

        # extract 1px-wide edges
        # gt = customSkeletonize(gt)
        gt = skeletonize(gt, method= "lee")

        # crop the lower part of the gt to match query
        gt = crop_gt_bottom_to_match_query_yspan(gt, query)

        # --- PCA ALIGNMENT ---
        aligned_gt, M, overlay, score = PCAlignment(query, gt, DT_Query, cq, vq, rq, draw_debug = True)

        # Show Results
        gt = (gt > 0).astype(np.uint8) * 255
        overlay_save = display_shapes(overlay, w, h, 0.001)
        ply_id = os.path.splitext(os.path.basename(path))[0]
        print(f"{ply_id}  ->  {score:.4f}")

        if overlay_save is not None:
                save_path = os.path.join(
                    project_root, "data/saved_images", 
                    f"Image_PCA_{ply_id}_session_{session_timestamp}.png"
                )
                cv2.imwrite(save_path, overlay_save)

        # Compute minimum Chamfer
        if score < best_score:
            best_score = score
            result = overlay
            matched_prealigned_gt = aligned_gt
            matched_gt = gt
            matched_id = os.path.splitext(os.path.basename(path))[0]
            T = M


    print("\nMatched Ply:")
    print(matched_id, "chamfer:", best_score)


    # -------------------------
    # ORB MATCHING LOOP
    # -------------------------

    # orb_overlay_init, orb_M, orb_overlay_final, orb_chamfer, orb_gt_aligned = ORBalignemnt(matched_prealigned_gt, query)

    # print("\n[ORB-matching]")
    # print("ORB chamfer:", orb_chamfer)
    # orb_overlay_init = display_shapes(orb_overlay_init, w, h, 0.5)
    # if orb_overlay_init is not None:
    #             save_path = os.path.join(
    #                 project_root, "data/saved_images", 
    #                 f"Image_ORB_{matched_id}_session_{session_timestamp}.png"
    #             )
    #             cv2.imwrite(save_path, orb_overlay_init)

    # if orb_chamfer > best_score:
    #     print("\n[ORB-matching]: Divergence detected, back to the PCA estimate")
    #     orb_gt_aligned = matched_prealigned_gt
    #     # reset orb_M to identity (rigid 2D affine)
    #     orb_M = np.array([
    #         [1.0, 0.0, 0.0],
    #         [0.0, 1.0, 0.0]
    #     ], dtype=np.float32)


    # ------------ ICP Final Alignment ------------

    # Align partia shape to full shape, then invert transformation
    R, t, final_overlay, chamfer = ICPrefinement(
        target_bin = matched_prealigned_gt, source_bin = query, 
        DT_query = DT_Query
    )

    print("\n[ICP-Refinement]")
    print("Final chamfer:", chamfer)
    final_overlay = display_shapes(final_overlay, w, h, 0.5)
    if final_overlay is not None:
                save_path = os.path.join(
                    project_root, "data/saved_images", 
                    f"Image_ICP_{matched_id}_session_{session_timestamp}.png"
                )
                cv2.imwrite(save_path, final_overlay)

    # -----------------------------------------
    # ------------ POSE ESTIMATION ------------
    # -----------------------------------------

    # ----- invert rigid transform -----
    R_inv = R.T
    t_inv = -R_inv @ t

    # get the final transform
    R_initial = T[:, :2]   # 2x2
    t_initial = T[:, 2]    # 2-vector
    # R_orb = orb_M[:, :2]
    # t_orb = orb_M[:, 2]

    # R_final = R_inv @ R_orb @ R_initial
    # t_final = R_inv @ (R_orb @ t_initial + t_orb) + t_inv
    R_final = R_inv  @ R_initial
    t_final = R_inv @ (t_initial) + t_inv

    # Table orientation 
    v_table = bl - tl            # direction vector
    norm = np.linalg.norm(v_table.astype(np.float32))
    if norm > 0:
        v_unit = v_table / norm
    else:
        v_unit = np.array([0.0, 0.0], dtype=np.float32)
    table_angle = np.arctan2(v_unit[0], v_unit[1])

    # rescale the tl corner  (Table reference point)
    tl_rescaled = np.array(tl, dtype=np.float32) * f
    tl_pt = (int(round(tl_rescaled[0])), int(round(tl_rescaled[1])))

    # --- make BGR canvases to draw on ---
    gt_vis = cv2.cvtColor(matched_gt.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    q_vis  = cv2.cvtColor(query.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    # draw table origin on query 
    cv2.circle(q_vis, tl_pt, 6, (0, 255, 0), -1)  # green point
    q_vis = tableAprilTagOrientation(q_vis, rvec, tl_pt)
    # direction from angle (wrt +y): [sin(a), cos(a)]
    # tbl_dir = np.array([np.sin(table_angle), np.cos(table_angle)], dtype=np.float32)
    # tbl_dir /= (np.linalg.norm(tbl_dir) + 1e-12)
    # tl = np.array(tl_pt, dtype=np.float32)  # (x, y)
    # p_tbl0 = tuple(np.round(tl).astype(int))
    L = 80.0  # arrow length in pixels
    # p_tbl1 = tuple(np.round(tl + L * tbl_dir).astype(int))
    # cv2.arrowedLine(q_vis, p_tbl0, p_tbl1, (0, 255, 0), 2, tipLength=0.25)

    # GT optimal picking
    p_new = R_final @ p_gt + t_final
    v_new = R_final @ v_gt 
    # --- draw points + arrows ---
    p0  = tuple(np.round(p_gt).astype(int))
    v_norm  = v_gt / (np.linalg.norm(v_gt) + 1e-12)
    p1  = tuple(np.round(p_gt + (L * v_norm)).astype(int))
    p0n = tuple(np.round(p_new).astype(int))
    vn  = v_new / (np.linalg.norm(v_new) + 1e-12)
    p1n = tuple(np.round(p_new + (L * vn)).astype(int))
    cv2.circle(gt_vis, p0, 6, (0, 255, 255), -1)  # yellow point
    cv2.arrowedLine(gt_vis, p0, p1, (0, 255, 255), 2, tipLength=0.25)
    cv2.circle(q_vis, p0n, 6, (255, 255, 0), -1)  # cyan point
    cv2.arrowedLine(q_vis, p0n, p1n, (255, 255, 0), 2, tipLength=0.25)

    # compute transformation angle
    picking_angle = np.arctan2(v_new[0], v_new[1])
    picking_z_rot = picking_angle - table_angle

    tl = np.array(tl_pt, dtype=np.float32)
    p_f  = np.array(p0n, dtype=np.float32)
    delta_px = p_f - tl

    # rescale in meters 
    dx_m = delta_px[0] * scale_gt
    dy_m = delta_px[1] * scale_gt

    print(f"\n [dx] = {dx_m} m, [dy] = {dy_m} m, [theta] = {picking_z_rot} rad\n")

    # Publish final Pose
    print("\n",matched_id,f"6D Table pose wrt camera: \nPosition={tvec.ravel()} \nOrientation={rvec.ravel()}\n")
    # transform from camera to TCP using extrinsics
    R_cam_obj, _ = cv2.Rodrigues(rvec)
    t_cam_obj = tvec.reshape(3, 1)
    # Apply Extrinsics: camera -> tcp  =>  p_tcp = R_cam2tcp * p_cam + t_cam2tcp
    R_tcp_cam = R_cam2tcp
    t_tcp_cam = t_cam2tcp.reshape(3, 1)
    # Compose: T_tcp_obj = T_tcp_cam * T_cam_obj
    R_tcp_obj = R_tcp_cam @ R_cam_obj
    t_tcp_obj = R_tcp_cam @ t_cam_obj + t_tcp_cam
    rvec_tcp_obj, _ = cv2.Rodrigues(R_tcp_obj)
    #print(f" [Pose] [{tag_ids[tl_idx]}] wrt tcp: \nPosition={t_tcp_obj.ravel()} \nOrientation={rvec_tcp_obj.ravel()}\n")
    publish_picking_pose(mqtt_client, rvec_tcp_obj, t_tcp_obj, dx_m, dy_m, picking_z_rot)

    gt_vis = display_shapes(gt_vis, w, h, 0.5)
    if gt_vis is not None:
            save_path = os.path.join(
                project_root, "data/saved_images", 
                f"Image_final_GT_session_{session_timestamp}.png"
            )
            cv2.imwrite(save_path, gt_vis)
    q_vis = display_shapes(q_vis, w, h, 0.5)
    if q_vis is not None:
            save_path = os.path.join(
                project_root, "data/saved_images", 
                f"Image_final_QUERY_session_{session_timestamp}.png"
            )
            cv2.imwrite(save_path, q_vis)

    proc = q_vis

    disp = proc

    # Reset activation flag after the pipeline
    mqtt_activation_event.clear()

    return disp, proc # proc is saved, disp is shown

def display_shapes(shape, w, h,time):
    shape = cv2.dilate(shape, np.ones((3,3), np.uint8), iterations=2)
    shape = cv2.resize(
        shape,
        (w, h),
        interpolation=cv2.INTER_NEAREST  # keep binary edges clean
    )
    display_image(shape)
    pump_gui_for(time)

    return shape

def display_image(frame):
    # resize to display
    disp = cv2.resize(
        frame,
        None,
        fx=0.5, # 50% size
        fy=0.5, # 50% size
        interpolation=cv2.INTER_AREA
    )
    cv2.imshow("Nano-M2590", disp)

def pump_gui_for(seconds):
    """
    Keep OpenCV GUI responsive for a given duration.

    Parameters
    ----------
    seconds : float
        Time duration in seconds.
    """
    t0 = time.monotonic()
    while (time.monotonic() - t0) < seconds:
        cv2.waitKey(1)

def print_ui_controls():
    print(
        "\n=== Vision UI Controls ===\n"
        "  [q] or [ESC] : Quit application\n"
        "  [s]          : Save current frame\n"
        "  [v]          : Trigger vision / detection algorithm\n"
        "===========================\n"
    )

if __name__ == "__main__":
    # Resolve paths relative to project root
    icalib_path = os.path.join(project_root, "calib/intrinsics", CAMERA_INTRINSICS_FILE)
    ecalib_path = os.path.join(project_root, "calib/extrinsics", CAMERA_EXTRINSICS_FILE)
    #model_path = os.path.join(project_root, "models", "yolov8n.pt")
    session_timestamp = datetime.now().strftime("%H_%M_%S__%d%m%Y")
    ns = 0 # counts saved frames
    
    # # Initialize Harvester for image aquisition (camera: Genie Nano-M2590 Mono)
    # h = Harvester()
    # h.add_file(CTI)
    # h.update()
    # ia = h.create(0)
    # nm = ia.remote_device.node_map
    # nm.PixelFormat.value = "Mono8" # Set Mono8 (in case Mono10 was pre-set instead)
    # ia.start()
    # last_good = None # contains last "good" frame when latest is corrupted

    # load camera stream from saved data if no camera is available 

    cam_dir = os.path.join(project_root, "data", "camera_stream")
    cam_paths = glob.glob(os.path.join(cam_dir, "*.png"))
    if not cam_paths:
        raise FileNotFoundError(f"No images found in {cam_dir}")
    img_path = random.choice(cam_paths)
    frame = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)


    # Load calibration
    K, dist, R, t = load_calibration(icalib_path, ecalib_path)
    print("\n[Genie Nano-M2590 Mono] intrinsics:")
    print("K =\n", K)
    print("dist =", dist.ravel())
    print("\n")
    print("\n[Camera->TCP] extrinsics:")
    print("R =\n", R)
    print("t =\n", t.ravel())
    print()

    # Initialize MQTT
    mqtt_client = create_mqtt_client()

    print_ui_controls()


    try:
        while True:
            # retrieve the latest frame from the camera buffer
            # with ia.fetch(timeout=1000) as buf:
            #     comp = buf.payload.components[0]
            #     # native resolution (2592×2048)
            #     raw_frame = np.frombuffer(comp.data, dtype=np.uint8)\
            #             .reshape(comp.height, comp.width).copy()
            #     # Due to lens vignetting we crop each frame, new resolution (1800x1500)
            #     frame = raw_frame[y_min:y_max, x_min:x_max]

            
            # # Harverster's Bug: every 3/4 frames the frame is entirely black.
            # # When that happens we keep showing the last good frame.
            # mx = int(frame.max())
            # if mx <= 2:
            #     # keep showing last_good so the window does not glitch
            #     if last_good is not None:
            #         display_image(last_good)
            # else:
            #     last_good = frame
            #     display_image(frame)

            display_image(frame)


            # UI commands [ToDelete] (just for designing the node)
            k = cv2.waitKey(1) & 0xFF

            if k in (27, ord('q')):     # ESC or 'q' → quit
                mqtt_termination_event.set()

            elif k == ord('s'):         # 's' → save frame
                ns += 1
                path = os.path.join(
                    project_root, "data/saved_images", 
                    f"Image_{ns}_session_{session_timestamp}.png"
                )
                cv2.imwrite(path, frame)
                print(f"Saved frame number: {ns}")
            
            elif k == ord('v'):         # 'v' → activate Vision algorithm
                name = input("Enter ply name: ").strip()
                if name:
                    print(f"Detection trigger received for: {name}")
                    ply_id = name   # store it somewhere accessible if needed
                    mqtt_activation_event.set()
                else:
                    print("No name provided. Detection not triggered.")

            # coordinator-triggered termination
            if mqtt_termination_event.is_set():
                print("Termination, shutting down Vision.")
                break
            
            # coordinator-triggered activation
            if mqtt_activation_event.is_set():
                img_path = os.path.join(cam_dir, ply_id + ".png")
                frame = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
                # run the Vision Pipeline
                disp, proc = run_pipeline(
                    latest_frame=frame,
                    K=K,
                    dist=dist,
                    R_cam2tcp = R,
                    t_cam2tcp = t,
                    #model_path=model_path,
                    mqtt_client=mqtt_client
                )

                if disp is not None and proc is not None:
                    # Display vision results
                    display_image(disp)
                    if proc is not None:
                        ns += 1
                        path = os.path.join(
                            project_root, "data/saved_images", 
                            f"Image_{ns}_session_{session_timestamp}.png"
                        )
                        cv2.imwrite(path, proc)
                        print(f"Saved frame number: {ns}")
                    pump_gui_for(2.0)
                
                img_path = random.choice(cam_paths)

            # coordinator-triggered calibration step
            if mqtt_calibration_event.is_set():
                # reset event from previous calib
                ext_calib_event.clear()
                # run the Calibration Pipeline
                disp = calibration_step(
                    latest_frame=frame,
                    K=K,
                    dist=dist,
                    mqtt_client=mqtt_client
                )
                # save image
                nc += 1
                path = os.path.join(saved_calib_images__dir_path, f"Image_{nc}.png")
                cv2.imwrite(path, disp)
                display_image(disp)
                pump_gui_for(1.0)
                # publish vision done to unlock planner
                publish_calib_done_vision(mqtt_client)


    finally:
        # Clean shutdowninitial_candidate
        # ia.stop()
        # ia.destroy()
        # h.reset()
        cv2.destroyAllWindows()
        mqtt_client.loop_stop()
        mqtt_client.disconnect()





