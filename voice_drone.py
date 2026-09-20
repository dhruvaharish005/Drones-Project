
import threading
import queue
import time
import os
from datetime import datetime

import numpy as np
import pybullet as p
from PIL import Image

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary


# ============================================================
# COMMAND QUEUE
# ============================================================

COMMANDS = queue.Queue()


def command_listener():
    """Read commands from the terminal."""
    while True:
        try:
            command = input(
                "\nCommand (takeoff/forward/backward/left/right/"
                "up/down/scan/stop/land/quit): "
            ).strip().lower()

            COMMANDS.put(command)

            if command in ("quit", "exit"):
                break

        except EOFError:
            COMMANDS.put("quit")
            break


# ============================================================
# 3D BUILDING ENVIRONMENT
# ============================================================

def create_building(env):
    """Create a simple building using PyBullet boxes."""

    client = env.CLIENT

    def add_box(position, half_extents, color, collision=True):
        collision_id = -1

        if collision:
            collision_id = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=half_extents,
                physicsClientId=client,
            )

        visual_id = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=half_extents,
            rgbaColor=color,
            physicsClientId=client,
        )

        return p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_id,
            baseVisualShapeIndex=visual_id,
            basePosition=position,
            physicsClientId=client,
        )

    # Main building
    add_box(
        position=[0, 4, 2],
        half_extents=[3, 0.5, 2],
        color=[0.65, 0.68, 0.72, 1],
    )

    # Rooftop
    add_box(
        position=[0, 4, 4.1],
        half_extents=[3.2, 0.7, 0.1],
        color=[0.25, 0.28, 0.32, 1],
    )

    # Front windows (visual only)
    for x in [-2, -1, 0, 1, 2]:
        for z in [1, 2.5, 3.5]:
            add_box(
                position=[x, 3.48, z],
                half_extents=[0.35, 0.025, 0.35],
                color=[0.15, 0.65, 0.95, 1],
                collision=False,
            )

    # Left neighboring structure
    add_box(
        position=[-4, 4, 1.5],
        half_extents=[0.5, 2, 1.5],
        color=[0.75, 0.72, 0.65, 1],
    )

    # Right neighboring structure
    add_box(
        position=[4, 4, 1.5],
        half_extents=[0.5, 2, 1.5],
        color=[0.75, 0.72, 0.65, 1],
    )

    # Rooftop room
    add_box(
        position=[0, 4, 4.7],
        half_extents=[0.8, 0.5, 0.5],
        color=[0.45, 0.48, 0.52, 1],
    )

    print("3D building environment created.")


# ============================================================
# CAMERA IMAGE CAPTURE
# ============================================================


def capture_drone_image(env, image_folder, image_number):
    """Capture and save an RGB image from the simulated drone camera."""

    client = env.CLIENT
    drone_id = env.DRONE_IDS[0]

    position, orientation = p.getBasePositionAndOrientation(
        drone_id,
        physicsClientId=client,
    )

    rotation_matrix = p.getMatrixFromQuaternion(orientation)

    forward = np.array([
        rotation_matrix[0],
        rotation_matrix[3],
        rotation_matrix[6],
    ])

    up = np.array([
        rotation_matrix[2],
        rotation_matrix[5],
        rotation_matrix[8],
    ])

    camera_position = np.array(position) + np.array([0, 0, 0.05])
    camera_target = camera_position + forward * 5.0

    view_matrix = p.computeViewMatrix(
        cameraEyePosition=camera_position.tolist(),
        cameraTargetPosition=camera_target.tolist(),
        cameraUpVector=up.tolist(),
    )

    width = 640
    height = 480

    projection_matrix = p.computeProjectionMatrixFOV(
        fov=70,
        aspect=width / height,
        nearVal=0.1,
        farVal=20,
    )

    image = p.getCameraImage(
        width=width,
        height=height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_TINY_RENDERER,
        physicsClientId=client,
    )

    # PyBullet returns RGBA pixel data in image[2].
    pixels = np.asarray(image[2], dtype=np.uint8)

    # Ensure the pixel array has the expected dimensions.
    if pixels.size != width * height * 4:
        print(
            "Camera image has unexpected size:",
            pixels.shape,
            "Expected:",
            (height, width, 4),
        )
        return

    rgba = pixels.reshape((height, width, 4))
    rgb = rgba[:, :, :3]

    os.makedirs(image_folder, exist_ok=True)

    filename = os.path.join(
        image_folder,
        f"scan_{image_number:03d}.png",
    )

    Image.fromarray(rgb, mode="RGB").save(filename)

    print(f"Captured image: {filename}")

# ============================================================
# DRONE SIMULATION
# ============================================================

def main():

    drone_model = DroneModel.CF2X
    num_drones = 1

    initial_xyzs = np.array([
        [0.0, 0.0, 0.1]
    ])

    initial_rpys = np.array([
        [0.0, 0.0, 0.0]
    ])

    # Create simulation
    env = CtrlAviary(
        drone_model=drone_model,
        num_drones=num_drones,
        initial_xyzs=initial_xyzs,
        initial_rpys=initial_rpys,
        physics=Physics.PYB,
        pyb_freq=240,
        ctrl_freq=48,
        gui=True,
        obstacles=False,
    )

    # Initialize PID controller
    controller = DSLPIDControl(
        drone_model=drone_model
    )

    # Reset first
    obs, info = env.reset()

    # Create building after reset
    create_building(env)

    # Target position and orientation
    target_pos = initial_xyzs[0].copy()
    target_rpy = initial_rpys[0].copy()

    running = True
    scanning = False

    scan_start = 0.0
    scan_start_yaw = 0.0
    scan_duration = 6.0

    # Image capture settings
    scan_image_folder = None
    scan_image_number = 0
    last_capture_time = 0.0
    capture_interval = 1.0

    print("\n================================")
    print("  DRONE SIMULATION STARTED")
    print("================================")
    print("3D building created in front of drone.")
    print("Type commands in this terminal.")
    print("Type 'quit' to close the simulation.")

    # Start terminal command listener
    threading.Thread(
        target=command_listener,
        daemon=True,
    ).start()

    try:
        while running:

            # --------------------------------------------
            # PROCESS COMMANDS
            # --------------------------------------------

            while not COMMANDS.empty():

                command = COMMANDS.get()

                if command in ("takeoff", "t"):
                    target_pos[2] = 1.0
                    print("Taking off")

                elif command in ("forward", "w"):
                    target_pos[1] += 0.5
                    print("Moving forward")

                elif command in ("backward", "back", "s"):
                    target_pos[1] -= 0.5
                    print("Moving backward")

                elif command in ("left", "a"):
                    target_pos[0] -= 0.5
                    print("Moving left")

                elif command in ("right", "d"):
                    target_pos[0] += 0.5
                    print("Moving right")

                elif command in ("up", "r"):
                    target_pos[2] += 0.3
                    print("Moving up")

                elif command in ("down", "f"):
                    target_pos[2] = max(
                        0.1,
                        target_pos[2] - 0.3,
                    )
                    print("Moving down")

                # ----------------------------------------
                # START SCAN
                # ----------------------------------------

                elif command in ("scan", "q"):

                    if not scanning:
                        scanning = True

                        scan_start = time.time()
                        scan_start_yaw = target_rpy[2]

                        scan_image_number = 0
                        last_capture_time = 0.0

                        timestamp = datetime.now().strftime(
                            "%Y%m%d_%H%M%S"
                        )

                        scan_image_folder = os.path.join(
                            "scan_images",
                            timestamp,
                        )

                        os.makedirs(
                            scan_image_folder,
                            exist_ok=True,
                        )

                        print("Starting 360-degree scan")
                        print(
                            f"Images will be saved in: "
                            f"{scan_image_folder}"
                        )

                # ----------------------------------------
                # STOP SCAN
                # ----------------------------------------

                elif command in ("stop", "e"):

                    scanning = False
                    print("Scan stopped")

                # ----------------------------------------
                # LAND
                # ----------------------------------------

                elif command in ("land", "l"):

                    scanning = False
                    target_pos[2] = 0.1

                    print("Landing")

                # ----------------------------------------
                # QUIT
                # ----------------------------------------

                elif command in ("quit", "exit"):

                    running = False

                elif command:
                    print("Unknown command:", command)

            # --------------------------------------------
            # SCAN ROTATION AND IMAGE CAPTURE
            # --------------------------------------------

            if scanning:

                elapsed = time.time() - scan_start

                progress = min(
                    elapsed / scan_duration,
                    1.0,
                )

                # Rotate through 360 degrees
                target_rpy[2] = (
                    scan_start_yaw
                    + 2.0 * np.pi * progress
                )

                # Capture one image every second
                if (
                    elapsed - last_capture_time
                    >= capture_interval
                    and progress < 1.0
                ):

                    capture_drone_image(
                        env,
                        scan_image_folder,
                        scan_image_number,
                    )

                    scan_image_number += 1
                    last_capture_time = elapsed

                # Finish scan
                if progress >= 1.0:

                    scanning = False
                    target_rpy[2] = scan_start_yaw

                    print("Scan complete")
                    print(
                        f"Total images captured: "
                        f"{scan_image_number}"
                    )

                    print(
                        f"Images saved in: "
                        f"{scan_image_folder}"
                    )

            # --------------------------------------------
            # PID CONTROL
            # --------------------------------------------

            action = np.zeros((num_drones, 4))

            action[0, :], _, _ = (
                controller.computeControlFromState(
                    control_timestep=env.CTRL_TIMESTEP,
                    state=obs[0],
                    target_pos=target_pos,
                    target_rpy=target_rpy,
                )
            )

            # --------------------------------------------
            # ADVANCE SIMULATION
            # --------------------------------------------

            obs, reward, terminated, truncated, info = (
                env.step(action)
            )

            time.sleep(1 / 48)

    except KeyboardInterrupt:
        print("\nStopping simulation")

    finally:
        env.close()
        print("Simulation closed")


if __name__ == "__main__":
    main()