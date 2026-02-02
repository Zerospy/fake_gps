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
while True:
    msg = m.recv_match(blocking=True)
    if not msg:
        continue
    msg_count += 1
    last_type = msg.get_type()
    t = msg.get_type()
    if t == 'RC_CHANNELS':
        last_rc = (msg.chan1_raw, msg.chan3_raw)
        print(f"RC: steer(ch1)={msg.chan1_raw} thr(ch3)={msg.chan3_raw}")
    elif t == 'SERVO_OUTPUT_RAW':
        last_servo = (msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw)
        print(f"SERVO: s1={msg.servo1_raw} s2={msg.servo2_raw} s3={msg.servo3_raw} s4={msg.servo4_raw}")

    now = time.time()
    if now - last_print >= 2.0:
        rc_s = f"rc={last_rc}" if last_rc else "rc=None"
        servo_s = f"servo={last_servo}" if last_servo else "servo=None"
        print(f"STATS: msgs={msg_count} last={last_type} {rc_s} {servo_s}")
        last_print = now
