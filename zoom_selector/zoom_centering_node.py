#!/usr/bin/env python3

# Main script for visual centering and zooming of an object using a Logitech C920 camera
# and controlling a Doosan robotic arm through ROS 2 services. The goal is to align the
# object in the camera's field of view and perform optimal zooming based on visual feedback.

import cv2
import numpy as np
import time
import os
import asyncio
import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveLine, MoveJoint

# === USER-DEFINED PARAMETERS (modify as needed) ===

# CAMERA_ID must match the correct /dev/videoX associated with the Logitech C920 camera.
# Use `v4l2-ctl --list-devices` and `v4l2-ctl -d /dev/videoX --list-ctrls` to determine it.
CAMERA_ID = 2

# Distance from the camera to the object in centimeters.
# IMPORTANT: Always measure accurately to avoid exceeding the robot's workspace.
OBJECT_DISTANCE_CM = 190

# Margin (in pixels) to prevent the object from reaching the image edges during zoom.
PIXEL_MARGIN = 20

# Reference image path (must match the object of interest).
REFERENCE_IMAGE_PATH = "/home/meic/Pictures/img_ref.jpg"

# === FIXED PARAMETERS ===

FOV_H_MIN = 35.38  # Minimum horizontal field of view in degrees (at max zoom)
FOV_H_MAX = 63.7   # Maximum horizontal FOV (at min zoom)
ZOOM_MIN = 100
ZOOM_MAX = 180
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
CENTER_X = IMAGE_WIDTH // 2
CENTER_Y = IMAGE_HEIGHT // 2
PIXEL_TO_MM_SCALE_FACTOR = 1.0
ZOOM_STEP_MM = 5.0
MAX_ZOOM_ITERATIONS = 10
SIFT = cv2.SIFT_create()
matcher = cv2.BFMatcher()

# Set camera zoom via v4l2 control
def set_zoom(level):
    os.system(f"v4l2-ctl -d /dev/video{CAMERA_ID} --set-ctrl zoom_absolute={level}")
    time.sleep(0.6)  # Allow zoom to stabilize

# Compute horizontal FOV from zoom level using linear interpolation
def compute_horizontal_fov(zoom):
    t = (zoom - ZOOM_MIN) / (ZOOM_MAX - ZOOM_MIN)
    return FOV_H_MAX - t * (FOV_H_MAX - FOV_H_MIN)

# Convert pixel displacement to real-world millimeters
def pixels_to_mm(dx_px, dy_px, zoom):
    fov = compute_horizontal_fov(zoom)
    fov_mm = 2 * OBJECT_DISTANCE_CM * np.tan(np.radians(fov / 2)) * 10
    mm_per_px = fov_mm / IMAGE_WIDTH
    return dx_px * mm_per_px * PIXEL_TO_MM_SCALE_FACTOR, dy_px * mm_per_px * PIXEL_TO_MM_SCALE_FACTOR

# Convert mm displacement to robot coordinate frame (X = right, Y = forward)
def convert_to_ros_coords(dx_mm, dy_mm):
    return -dy_mm, dx_mm

# Detect object displacement via SIFT feature matching and homography
def calculate_displacement(img, kp_ref, des_ref):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    kp_img, des_img = SIFT.detectAndCompute(gray, None)
    if des_img is None:
        return None, None, None, None, None

    matches = matcher.knnMatch(des_ref, des_img, k=2)
    good = [m for m, n in matches if m.distance < 0.7 * n.distance]
    if len(good) < 4:
        return None, None, None, None, None

    src_pts = np.float32([kp_ref[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst_pts = np.float32([kp_img[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    H, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
    if H is None:
        return None, None, None, None, None

    h, w = img_ref.shape
    corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
    projected = cv2.perspectiveTransform(corners, H)
    cx = int(np.mean(projected[:, 0, 0]))
    cy = int(np.mean(projected[:, 0, 1]))
    H = H / H[2, 2]
    angle_z = np.degrees(np.arctan2(H[1, 0], H[0, 0]))
    dx = cx - CENTER_X
    dy = cy - CENTER_Y
    return int(dx), int(dy), good, angle_z, projected

# Display the image with the center of the object (red) and camera (green)
def display_centers(img, dx, dy):
    vis = img.copy()
    cv2.circle(vis, (CENTER_X, CENTER_Y), 5, (0, 255, 0), -1)  # camera center
    cv2.circle(vis, (CENTER_X + dx, CENTER_Y + dy), 5, (0, 0, 255), -1)  # object center
    cv2.imshow("Camera View", vis)
    cv2.waitKey(1)

# Verify that the projected bounding box is fully within the image margins
def is_within_bounds(projected):
    x_proj = projected[:, 0, 0]
    y_proj = projected[:, 0, 1]
    return (
        x_proj.min() > PIXEL_MARGIN and x_proj.max() < IMAGE_WIDTH - PIXEL_MARGIN and
        y_proj.min() > PIXEL_MARGIN and y_proj.max() < IMAGE_HEIGHT - PIXEL_MARGIN
    )

# Convert pixel displacement to rotation angles in degrees
def pixel_to_angle(dx_px, dy_px, zoom):
    fov_h_deg = compute_horizontal_fov(zoom)
    fov_v_deg = fov_h_deg * IMAGE_HEIGHT / IMAGE_WIDTH
    angle_x = (dx_px / IMAGE_WIDTH) * fov_h_deg
    angle_y = (dy_px / IMAGE_HEIGHT) * fov_v_deg
    return -angle_y, -angle_x

# Perform linear movement using MoveLine ROS 2 service
async def move_robot(node, client, pos):
    req = MoveLine.Request()
    req.pos = pos
    req.vel = [20.0, 20.0]
    req.acc = [40.0, 40.0]
    req.time = 0.0
    req.radius = 0.0
    req.ref = 1
    req.mode = 1
    req.blend_type = 0
    req.sync_type = 0
    future = client.call_async(req)
    print("🤖 Sending MoveLine command...")
    while rclpy.ok() and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    if future.result() is not None:
        print("✅ Move executed successfully.")
    else:
        print("❌ Move failed.")

# Perform rotation of joint 4 using MoveJoint ROS 2 service
async def move_joint_4(node, client, delta_deg):
    req = MoveJoint.Request()
    req.pos = [0.0, 0.0, 0.0, delta_deg, 0.0, 0.0]
    req.vel = 10.0
    req.acc = 20.0
    req.time = 0.0
    req.radius = 0.0
    req.mode = 1
    req.blend_type = 0
    req.sync_type = 0
    future = client.call_async(req)
    while rclpy.ok() and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    if future.result() and future.result().success:
        print("✅ Joint 4 movement successful.")
    else:
        print("❌ Joint 4 movement failed.")

# Main asynchronous logic loop
async def async_main():
    rclpy.init()
    node = rclpy.create_node('zoom_centering_node')
    client = node.create_client(MoveLine, '/dsr01/motion/move_line')
    joint_client = node.create_client(MoveJoint, '/dsr01/motion/move_joint')

    for cli in [client, joint_client]:
        while not cli.wait_for_service(timeout_sec=1.0):
            print("⏳ Waiting for movement services...")

    global img_ref
    img_ref = cv2.imread(REFERENCE_IMAGE_PATH, cv2.IMREAD_GRAYSCALE)
    if img_ref is None:
        raise FileNotFoundError("❌ Reference image not found. Please check the file path.")
    kp_ref, des_ref = SIFT.detectAndCompute(img_ref, None)

    cap = cv2.VideoCapture(CAMERA_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)
    if not cap.isOpened():
        raise RuntimeError("❌ Could not open camera device.")

    cv2.namedWindow("Camera View", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera View", 960, 540)
    set_zoom(ZOOM_MIN)
    print("📷 System active. Starting capture...")

    # First visual centering phase
    while True:
        ret, frame = cap.read()
        if not ret:
            continue  # Skip if frame not captured

        dx, dy, good, ang_z, _ = calculate_displacement(frame, kp_ref, des_ref)
        cv2.imshow("Camera View", frame)

        if dx is not None:
            display_centers(frame, dx, dy)  # Draw red and green centers

            # Convert pixel displacement to millimeters using FOV and distance
            dx_mm, dy_mm = pixels_to_mm(dx, dy, ZOOM_MIN)
            x_ros, y_ros = convert_to_ros_coords(dx_mm, dy_mm)

            print(f"Initial dx: {dx} dy: {dy}")

            # Send movement command to robot to align the object visually
            await move_robot(node, client, [x_ros, y_ros, 0.0, 0.0, 0.0, ang_z])
            time.sleep(2.0)

            # Measure post-move visual error by averaging several frames
            error_samples_px = []
            for _ in range(6):
                cap.read()  # Skip old frame
                time.sleep(0.1)
                ret2, frame2 = cap.read()
                dx2, dy2, _, _, _ = calculate_displacement(frame2, kp_ref, des_ref)
                if dx2 is not None:
                    error_samples_px.append(np.linalg.norm([dx2, dy2]))

            if len(error_samples_px) == 0:
                print("❌ Failed to calculate new error after movement.")
                break

            avg_new_error = np.mean(error_samples_px)
            initial_error = np.linalg.norm([dx, dy])
            print(f"📏 Initial error: {initial_error:.2f}px | New average error: {avg_new_error:.2f}px")

            # If significant residual error remains, apply angular correction
            if avg_new_error > 0.05 * initial_error:
                pitch, roll = pixel_to_angle(dx2, dy2, ZOOM_MIN)
                print(f"🔄 Applying Roll={roll:.2f}°, Pitch={pitch:.2f}°")
                await move_robot(node, client, [0.0, 0.0, 0.0, 0.0, pitch, 0.0])
                await move_joint_4(node, joint_client, roll)
                time.sleep(2.0)
            break

        if cv2.waitKey(1) == 27:
            return  # Exit on ESC key

    # Linear zoom loop until no further improvement is observed
    current_zoom = ZOOM_MIN
    z_offset = 0.0
    no_zoom_improvement = 0

    for z_iter in range(MAX_ZOOM_ITERATIONS):
        prev_zoom = current_zoom

        while current_zoom + 2 <= ZOOM_MAX:
            current_zoom += 2
            set_zoom(current_zoom)
            print(f"🔍 Zoom adjusted to: {current_zoom}")
            ret, frame = cap.read()
            if not ret:
                continue

            dx2, dy2, good2, _, projected = calculate_displacement(frame, kp_ref, des_ref)

            if dx2 is None or len(good2) < 8 or not is_within_bounds(projected):
                current_zoom -= 2  # Revert to previous valid zoom
                break

            display_centers(frame, dx2, dy2)

        if current_zoom <= prev_zoom:
            no_zoom_improvement += 1
        else:
            no_zoom_improvement = 0

        if no_zoom_improvement >= 2:
            print("🛑 Zoom no longer improves image quality. Stopping.")
            break

        #z_offset += Z_INCREMENT_MM
        # Movement in Z is commented out for safety, enable if vertical motion is needed
        # await move_robot(node, client, [0.0, 0.0, z_offset, 0.0, 0.0, 0.0])
        time.sleep(2)

    print("✅ Final zoom level reached. Press [ESC] to exit.")

    # Final evaluation of projected object size and visual error
    ret, frame = cap.read()
    if ret:
        dx_final, dy_final, good, _, projected = calculate_displacement(frame, kp_ref, des_ref)
        if projected is not None:
            width_proj = np.linalg.norm(projected[0] - projected[1])
            height_proj = np.linalg.norm(projected[0] - projected[3])
            h_ref, w_ref = img_ref.shape

            width_ref = w_ref
            height_ref = h_ref

            width_error_pct = abs(width_proj - width_ref) / width_ref * 100
            height_error_pct = abs(height_proj - height_ref) / height_ref * 100
            pixel_error = np.linalg.norm([dx_final, dy_final])

            print("📐 Final size comparison with reference:")
            print(f"  🔸 Projected width: {width_proj:.2f}px (Reference: {width_ref}px)")
            print(f"  🔸 Projected height: {height_proj:.2f}px (Reference: {height_ref}px)")
            print(f"  ➕ Width error: {width_error_pct:.2f}% | Height error: {height_error_pct:.2f}%")
            print(f"  🎯 Pixel center error: {pixel_error:.2f}px")
            print(f"     Angle error: {ang_z:.2f}°") 
            print(f" dx_final: {dx_final} dy_final: {dy_final}")

    # Show final image until user exits
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        dx_final, dy_final, *_ = calculate_displacement(frame, kp_ref, des_ref)
        if dx_final is not None:
            display_centers(frame, dx_final, dy_final)
        else:
            cv2.imshow("Camera View", frame)

        key = cv2.waitKey(1)
        if key == 27:
            print("👋 Exiting...")
            break

    # Cleanup on exit
    cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

def main():
    asyncio.run(async_main())




