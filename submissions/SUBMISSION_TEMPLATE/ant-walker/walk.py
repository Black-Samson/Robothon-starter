import mujoco
import mujoco.viewer
import numpy as np
import json
import time

# ─────────────────────────────────────────
# Load model
# ─────────────────────────────────────────
model = mujoco.MjModel.from_xml_path("ant.xml")
data  = mujoco.MjData(model)

# ─────────────────────────────────────────
# Movement state  — TOGGLE mode
# (Windows MuJoCo only fires one event per key with no action/release info,
#  so we can't detect "key held". Instead each key TOGGLES its direction on/off.
#  Pressing the same key again cancels it. Pressing an opposite key switches.)
# ─────────────────────────────────────────
# speed_dir: -1 = backward, 0 = stopped, +1 = forward
# turn_dir:  -1 = left,     0 = straight, +1 = right
speed_dir   = 0
turn_dir    = 0
autopilot   = False
do_reset    = False

# MuJoCo/GLFW key codes
KEY_W   = 87;  KEY_S = 83
KEY_A   = 65;  KEY_D = 68
KEY_R   = 82;  KEY_TAB = 258; KEY_SPACE = 32

# MuJoCo's own shadow-toggle shortcut is also bound to D internally.
# We cannot prevent that from inside key_callback on Windows.
# Solution: we REMAP turn-right to the RIGHT ARROW key instead of D.
KEY_LEFT  = 263
KEY_RIGHT = 262
KEY_UP    = 265
KEY_DOWN  = 264

def key_callback(*args):
    global speed_dir, turn_dir, autopilot, do_reset

    if len(args) == 0:
        return
    keycode = args[0]
    # On Windows only 1 arg arrives (no action). On Linux/Mac 2+ args arrive.
    # We only act on PRESS (action==1) or when action is unknown (Windows).
    if len(args) >= 2:
        action = args[1]
        if action not in (1, 2):   # ignore RELEASE (0) and anything else
            return

    # ── Movement toggles ──────────────────────────────────────────
    if keycode in (KEY_W, KEY_UP):
        speed_dir = 0 if speed_dir == 1 else 1      # toggle forward
        turn_dir  = 0                                # straighten when changing speed
    elif keycode in (KEY_S, KEY_DOWN):
        speed_dir = 0 if speed_dir == -1 else -1    # toggle backward
        turn_dir  = 0
    elif keycode in (KEY_A, KEY_LEFT):
        turn_dir  = 0 if turn_dir == -1 else -1     # toggle left
    elif keycode in (KEY_D, KEY_RIGHT):
        turn_dir  = 0 if turn_dir == 1 else 1       # toggle right
    elif keycode == KEY_SPACE:
        speed_dir = 0;  turn_dir = 0                # STOP immediately
    elif keycode == KEY_TAB:
        autopilot = not autopilot
        speed_dir = 0;  turn_dir = 0                # clear manual state
        print(f"\n  [AUTO-PILOT {'ON' if autopilot else 'OFF'}]\n")
    elif keycode == KEY_R:
        do_reset = True

# ─────────────────────────────────────────
# Actuator layout
# 0=hip1(FL) 1=ankle1(FL)  2=hip2(FR) 3=ankle2(FR)
# 4=hip3(BL) 5=ankle3(BL)  6=hip4(BR) 7=ankle4(BR)
# ─────────────────────────────────────────
STAND = np.array([0.0, 0.5,  0.0, -0.5,  0.0, -0.5,  0.0, 0.5])
INIT_QPOS = None

def compute_gait(t, speed=1.0, turn=0.0):
    ctrl = STAND.copy()
    if speed == 0.0 and turn == 0.0:
        return ctrl

    freq  = 2.0
    phase = 2 * np.pi * freq * t
    fwd   = np.sign(speed) if speed != 0 else 1.0
    mag   = min(abs(speed), 1.0)
    hip_amp, ankle_amp = 0.8 * mag, 0.6 * mag

    a, b = np.sin(phase), np.sin(phase + np.pi)

    ctrl[0] =  hip_amp * a * fwd + turn * 0.5
    ctrl[1] =  0.5 + ankle_amp * a
    ctrl[2] = -hip_amp * b * fwd - turn * 0.5
    ctrl[3] = -0.5 - ankle_amp * b
    ctrl[4] = -hip_amp * b * fwd + turn * 0.5
    ctrl[5] = -0.5 - ankle_amp * b
    ctrl[6] =  hip_amp * a * fwd - turn * 0.5
    ctrl[7] =  0.5 + ankle_amp * a

    return np.clip(ctrl, -1.0, 1.0)

# ─────────────────────────────────────────
# Multi-waypoint goals
# ─────────────────────────────────────────
WAYPOINTS = [
    np.array([2.0, 2.0]),
    np.array([4.0, 0.0]),
    np.array([4.0, 4.0]),
]
WAYPOINT_LABELS = ["Checkpoint 1 (2,2)", "Checkpoint 2 (4,0)", "FINAL GOAL (4,4)"]
GOAL_RADIUS = 0.6
current_wp  = 0
wp_times    = []

def check_waypoint(pos):
    if current_wp >= len(WAYPOINTS):
        return False
    return np.linalg.norm(pos[:2] - WAYPOINTS[current_wp]) < GOAL_RADIUS

# ─────────────────────────────────────────
# Auto-pilot
# ─────────────────────────────────────────
def get_yaw(qpos):
    qw, qx, qy, qz = qpos[3], qpos[4], qpos[5], qpos[6]
    return np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

def autopilot_control(qpos):
    if current_wp >= len(WAYPOINTS):
        return 0.0, 0.0
    target = WAYPOINTS[current_wp]
    diff   = target - qpos[:2]
    dist   = np.linalg.norm(diff)
    if dist < GOAL_RADIUS:
        return 0.0, 0.0
    desired_yaw = np.arctan2(diff[1], diff[0])
    err  = (desired_yaw - get_yaw(qpos) + np.pi) % (2*np.pi) - np.pi
    turn = np.clip(err / np.pi, -1.0, 1.0)
    spd  = np.clip(dist / 2.0,  0.2,  1.0)
    return spd, turn

# ─────────────────────────────────────────
# Reset
# ─────────────────────────────────────────
def reset_ant():
    global current_wp, wp_times, speed_dir, turn_dir
    mujoco.mj_resetData(model, data)
    if INIT_QPOS is not None:
        data.qpos[:] = INIT_QPOS
    data.ctrl[:] = STAND
    mujoco.mj_forward(model, data)
    current_wp = 0
    wp_times   = []
    speed_dir  = 0
    turn_dir   = 0
    print("\n  [RESET] Back to start.\n")

# ─────────────────────────────────────────
# Logger
# ─────────────────────────────────────────
log = {
    "session_start":        time.strftime("%Y-%m-%d %H:%M:%S"),
    "commands_issued":      [],
    "distance_travelled_m": 0.0,
    "waypoints_reached":    0,
    "waypoint_times_s":     [],
    "goal_reached":         False,
    "time_to_goal_s":       None,
    "total_sim_steps":      0,
}
last_pos      = None
last_cmd      = ""
session_start = time.time()

def record_command(cmd):
    global last_cmd
    if cmd != last_cmd:
        log["commands_issued"].append({"command": cmd,
                                       "time_s": round(time.time()-session_start, 2)})
        last_cmd = cmd

# ─────────────────────────────────────────
# Main
# ─────────────────────────────────────────
print("\n" + "="*58)
print("  ROBOTHON 2026 — Ant Walker & Explorer")
print("="*58)
print("  W / UP    — toggle forward")
print("  S / DOWN  — toggle backward")
print("  A / LEFT  — toggle turn left")
print("  D / RIGHT — toggle turn right   (press again to stop)")
print("  SPACE     — stop all movement")
print("  R         — reset ant to start")
print("  TAB       — toggle AUTO-PILOT")
print()
print("  Note: each key TOGGLES on/off. Press it again to stop.")
print()
print("  Waypoints:")
for i, lbl in enumerate(WAYPOINT_LABELS):
    marker = ["🔵","🟠","🟡"][i]
    print(f"    {marker}  {lbl}")
print("="*58)
print("\nSettling physics...\n")

with mujoco.viewer.launch_passive(model, data,
                                  key_callback=key_callback) as viewer:

    viewer.cam.type        = mujoco.mjtCamera.mjCAMERA_TRACKING
    viewer.cam.trackbodyid = model.body("torso").id
    viewer.cam.distance    = 6.0
    viewer.cam.elevation   = -28

    sim_time = 0.0
    dt       = float(model.opt.timestep)

    for _ in range(int(1.5 / dt)):
        data.ctrl[:] = STAND
        mujoco.mj_step(model, data)
        viewer.sync()
        sim_time += dt

    INIT_QPOS = data.qpos.copy()
    last_pos  = data.qpos[:3].copy()

    print("Ant is up!")
    print("Use W/S/A/D or arrow keys (each press toggles on/off).")
    print("SPACE = stop.  TAB = auto-pilot.  R = reset.\n")
    print(f"First target: {WAYPOINT_LABELS[0]}\n")

    while viewer.is_running():

        # ── Reset ──────────────────────────────
        if do_reset:
            reset_ant()
            sim_time   = 0.0
            do_reset   = False  # type: ignore  (module-level var)
            # can't assign to nonlocal here; use a workaround:
            import sys
            # just clear via the globals dict
            globals()["do_reset"] = False
            continue

        # ── Controls ───────────────────────────
        if autopilot:
            spd, trn = autopilot_control(data.qpos)
            cmd = f"auto→wp{current_wp+1}" if current_wp < len(WAYPOINTS) else "auto→done"
        else:
            spd  = float(speed_dir)
            trn  = float(turn_dir)
            parts = []
            if speed_dir ==  1: parts.append("fwd")
            if speed_dir == -1: parts.append("bwd")
            if turn_dir  == -1: parts.append("left")
            if turn_dir  ==  1: parts.append("right")
            cmd = "+".join(parts) if parts else "idle"

        record_command(cmd)

        data.ctrl[:] = compute_gait(sim_time, spd, trn)
        mujoco.mj_step(model, data)
        viewer.sync()

        cur_pos = data.qpos[:3].copy()
        if last_pos is not None:
            log["distance_travelled_m"] += float(np.linalg.norm(cur_pos - last_pos))
        last_pos = cur_pos
        log["total_sim_steps"] += 1
        sim_time += dt

        # ── Waypoint detection ─────────────────
        if current_wp < len(WAYPOINTS) and check_waypoint(cur_pos):
            elapsed = round(time.time() - session_start, 2)
            wp_times.append(elapsed)
            log["waypoints_reached"] += 1
            log["waypoint_times_s"].append(elapsed)
            print(f"\n  ✅  {WAYPOINT_LABELS[current_wp]} — {elapsed}s")
            current_wp += 1
            if current_wp >= len(WAYPOINTS):
                log["goal_reached"]   = True
                log["time_to_goal_s"] = elapsed
                print(f"\n  🎯  ALL WAYPOINTS DONE in {elapsed}s!\n")
            else:
                print(f"  ➡  Next: {WAYPOINT_LABELS[current_wp]}\n")

    # ── Save log ───────────────────────────────
    log["distance_travelled_m"] = round(log["distance_travelled_m"], 3)
    log["session_duration_s"]   = round(time.time() - session_start, 2)
    with open("session_log.json", "w") as f:
        json.dump(log, f, indent=2)

    print("\n" + "="*58)
    print("  Session complete!")
    print(f"  Distance  : {log['distance_travelled_m']} m")
    print(f"  Duration  : {log['session_duration_s']} s")
    print(f"  Waypoints : {log['waypoints_reached']} / {len(WAYPOINTS)}")
    print(f"  Goal      : {'✅ Reached' if log['goal_reached'] else '❌ Not reached'}")
    print(f"  Commands  : {len(log['commands_issued'])}")
    print("  Saved → session_log.json")
    print("="*58 + "\n")