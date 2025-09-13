import cv2
import time
import math
from typing import Tuple, List
from djitellopy import Tello

# External modules you already have
import tello_tracking
import path_planner
import avoid

############################################################
# Utility helpers
############################################################

def clamp_int(v: float, lo: int = -100, hi: int = 100) -> int:
    """Clamp and convert to int for Tello RC commands."""
    if v is None:
        return 0
    if v > hi:
        v = hi
    elif v < lo:
        v = lo
    return int(v)


def safe_frame(read) -> Tuple[bool, any]:
    """Grab the most recent frame safely; return (ok, frame)."""
    try:
        frame = read.frame
        if frame is None:
            return False, None
        return True, frame
    except Exception:
        return False, None


############################################################
# Main
############################################################

def main():
    # --- OpenCV perf knobs ---
    # Use a single thread to reduce contention/latency jitter.
    try:
        cv2.setNumThreads(1)
    except Exception:
        pass

    # ------------------------------
    # PATH DEFINITIONS
    # ------------------------------
    start_1_X, start_1_Y, end_1_X, end_1_Y = 0, 0, 50, 0
    path1 = [start_1_X, start_1_Y, end_1_X, end_1_Y]

    # Other drone path (for collision sim)
    path2 = [0, 30, 50, 80]

    # Drone kinematic state: [x, y, yaw_deg]
    drone_pos = [path1[0], path1[1], 0]

    # Desired body-frame velocity command (vx, vy) & yaw rate
    drone_cmd = [0, 0, 0]

    # Instantiate your modules
    cv_mod = tello_tracking.CV()
    planner = path_planner.PathPlan(path1[0], path1[2], path1[1], path1[3], drone_pos[2])
    collision = avoid.Avoid(path1, path2)

    # Goal flag
    reached_goal = False

    # ------------------------------
    # TELLO SETUP
    # ------------------------------
    tello = Tello()
    tello.connect()
    print(f"battery: {tello.get_battery()}")

    # Video
    tello.streamon()
    reader = tello.get_frame_read()

    # Takeoff
    tello.takeoff()

    # Climb to a nominal height smoothly (non-blocking approach)
    nominal_up_cmd = 25  # modest vertical speed command
    tello.send_rc_control(0, 0, clamp_int(nominal_up_cmd), 0)
    time.sleep(1.0)
    tello.send_rc_control(0, 0, 0, 0)

    # Track an estimated height in arbitrary units (RC units-integrated)
    est_height = 0.0
    nominal_height = 50.0  # target height in same units as est_height
    vz_cmd = 0

    # Timing
    last_t = time.time()

    # UI
    show_video = True
    if show_video:
        cv2.namedWindow("Drone 1", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Drone 1", 640, 480)

    # ------------------------------
    # INITIAL ORIENTATION (center on human)
    # ------------------------------
    facing_human = False

    while not facing_human:
        ok, frame = safe_frame(reader)
        if not ok:
            time.sleep(0.01)
            continue

        # (turn_cmd, detections)
        turn_cmd, detections = cv_mod.center_subject(frame)

        # Fallback: if no human detected, keep rotating gently
        if turn_cmd == 0:
            turn_cmd = 25
        elif turn_cmd == 1:
            facing_human = True
            turn_cmd = 0

        # Draw detections (cheap overlays)
        if show_video:
            for det in detections:
                x, y, w, h = det.get("box", (0, 0, 0, 0))
                label = f"{det.get('label','obj')} {det.get('confidence', 0):.2f}"
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, label, (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow("Drone 1", frame)
            cv2.waitKey(1)

        # Send yaw-only turn during orientation stage
        tello.send_rc_control(0, 0, 0, clamp_int(turn_cmd))
        time.sleep(0.03)  # small yield; keeps SDK happy without adding big latency

    # Stop yawing
    tello.send_rc_control(0, 0, 0, 0)
    print("\n\nnow moving paths\n\n")

    # ------------------------------
    # PATH PLANNING LOOP (~50 Hz)
    # ------------------------------
    start_time = time.time()
    while time.time() - start_time < 60.0:
        t = time.time()
        dt = max(1e-3, t - last_t)
        last_t = t

        ok, frame = safe_frame(reader)
        if not ok:
            # If video hiccups, keep flying with last command but avoid stale CPU burn
            time.sleep(0.005)
            continue

        # --- Perception (cheap) ---
        turn_cmd, detections = cv_mod.center_subject(frame)
        if turn_cmd == 1:
            # Already centered
            turn_cmd = 0
        yaw_cmd = clamp_int(turn_cmd)

        # --- State update (read cached state from SDK) ---
        try:
            drone_pos[2] = -float(tello.get_yaw())  # yaw in degrees; negate to match your convention
        except Exception:
            # If state packet missed, keep last yaw
            pass

        # --- Dead-reckon planar position based on last body-frame cmd ---
        # Convert body velocities to world deltas using current yaw
        # NOTE: RC units are arbitrary; we integrate for relative motion only.
        # drone_cmd = [vx_body, vy_body, yaw_rate_cmd]
        vx_body, vy_body = float(drone_cmd[0]), float(drone_cmd[1])

        # Resolve to world frame
        yaw_rad = math.radians(drone_pos[2] % 360)
        # Body x is to the right, y is forward in Tello's RC convention
        # djitellopy send_rc_control = (left_right, forward_back, up_down, yaw)
        # Your code used movement[0] as x and movement[1] as y, so we keep that mapping.
        dx = (abs(vx_body) * math.cos(yaw_rad + math.pi/2 * (-1 if vx_body > 0 else 1))
              + vy_body * math.cos(yaw_rad)) * dt
        dy = (abs(vx_body) * math.sin(yaw_rad + math.pi/2 * (-1 if vx_body > 0 else 1))
              + vy_body * math.sin(yaw_rad)) * dt
        drone_pos[0] += dx
        drone_pos[1] += dy

        # --- Collision/Altitude manager (very light-weight) ---
        # Update naive height estimate from commanded vertical speed
        est_height += float(vz_cmd) * dt

        col = collision.detect_collision(drone_pos[0], drone_pos[1])
        if col == "collision":
            vz_cmd = 30
        else:
            # Relax toward nominal height with a tiny P controller
            if est_height > 1.1 * nominal_height:
                vz_cmd = -20
            elif est_height < 0.9 * nominal_height:
                vz_cmd = 20
            else:
                vz_cmd = 0

        # --- Planner step ---
        # Returns desired body-frame motion; ensure ints and sane ranges
        desired = planner.move_towards_goal(drone_pos[0], drone_pos[1], drone_pos[2], reached_goal)
        # Expecting [vx, vy, <optional>] where vx==0.1 is their goal sentinel
        if isinstance(desired, (list, tuple)) and len(desired) >= 2:
            vx, vy = desired[0], desired[1]
        else:
            vx, vy = 0, 0

        if vx == 0.1:
            reached_goal = True
            vx, vy = 0, 0

        # Save for dead-reckoning next tick
        drone_cmd[0] = vx
        drone_cmd[1] = vy

        # Compose RC command (LR, FB, UD, Yaw). Cast -> int & clamp.
        lr = clamp_int(vx)
        fb = clamp_int(vy)
        ud = clamp_int(vz_cmd)
        yw = yaw_cmd

        tello.send_rc_control(lr, fb, ud, yw)

        # --- Minimal UI draw (optional) ---
        if show_video:
            for det in detections:
                x, y, w, h = det.get("box", (0, 0, 0, 0))
                label = f"{det.get('label','obj')} {det.get('confidence', 0):.2f}"
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, label, (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow("Drone 1", frame)
            cv2.waitKey(1)

        # Soft-rate limit around 50 Hz
        time.sleep(0.02)

    # End loop

    # ------------------------------
    # Teardown
    # ------------------------------
    try:
        tello.send_rc_control(0, 0, 0, 0)
        time.sleep(0.5)
    finally:
        if show_video:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        try:
            tello.land()
        except Exception:
            pass
        try:
            tello.streamoff()
        except Exception:
            pass
        tello.end()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nManual stop requested. Landing drone...")
        # Best-effort cleanup happens in main's finally
