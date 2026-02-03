from pymavlink import mavutil
import os
import time

IN_UDP_PORT = int(os.getenv("IN_UDP_PORT", "14560"))
IN_MAVLINK = os.getenv("IN_MAVLINK")
in_conn = IN_MAVLINK or f'udpin:0.0.0.0:{IN_UDP_PORT}'
m = mavutil.mavlink_connection(in_conn)

hb = m.wait_heartbeat(timeout=5)
if hb:
    print(f"HEARTBEAT ok on {in_conn}")
else:
    print(f"No HEARTBEAT on {in_conn} (5s). If Mission Planner is your only link, enable UDP forwarding to this port.")

last_print = time.time()
msg_count = 0
last_type = None
last_rc = None
last_servo = None
current_mode = None
armed = None
last_statustext = None
last_src = None
PRINT_UNKNOWN = os.getenv("PRINT_UNKNOWN", "0").strip() in {"1", "true", "TRUE", "yes", "YES"}
while True:
    msg = m.recv_match(blocking=True)
    if not msg:
        continue
    msg_count += 1
    last_type = msg.get_type()
    try:
        last_src = (msg.get_srcSystem(), msg.get_srcComponent())
    except Exception:
        last_src = None
    t = msg.get_type()
    if t == 'HEARTBEAT':
        try:
            current_mode = mavutil.mode_string_v10(msg)
        except Exception:
            current_mode = None
        try:
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        except Exception:
            armed = None
    elif t == 'STATUSTEXT':
        try:
            last_statustext = msg.text
            print(f"STATUSTEXT: {msg.text}")
        except Exception:
            pass
    elif t == 'RC_CHANNELS':
        last_rc = (msg.chan1_raw, msg.chan3_raw)
        print(f"RC: steer(ch1)={msg.chan1_raw} thr(ch3)={msg.chan3_raw}")
    elif t == 'SERVO_OUTPUT_RAW':
        last_servo = (
            msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw,
            msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw,
        )
        print(
            "SERVO: "
            f"s1={msg.servo1_raw} s2={msg.servo2_raw} s3={msg.servo3_raw} s4={msg.servo4_raw} "
            f"s5={msg.servo5_raw} s6={msg.servo6_raw} s7={msg.servo7_raw} s8={msg.servo8_raw}"
        )
    elif PRINT_UNKNOWN:
        # Dump any other message types for debugging MAVLink routing/stream rates.
        try:
            print(f"MSG: {t} src={last_src} {msg.to_dict()}")
        except Exception:
            print(f"MSG: {t} src={last_src}")

    now = time.time()
    if now - last_print >= 2.0:
        rc_s = f"rc={last_rc}" if last_rc else "rc=None"
        servo_s = f"servo={last_servo}" if last_servo else "servo=None"
        mode_s = f"mode={current_mode}" if current_mode else "mode=?"
        armed_s = "armed=?" if armed is None else f"armed={int(armed)}"
        text_s = f"text={last_statustext!r}" if last_statustext else "text=None"
        src_s = f"src={last_src}" if last_src else "src=?"
        print(f"STATS: msgs={msg_count} last={last_type} {src_s} {mode_s} {armed_s} {rc_s} {servo_s} {text_s}")
        last_print = now
