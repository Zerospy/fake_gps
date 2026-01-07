from pymavlink import mavutil
m = mavutil.mavlink_connection('udpin:0.0.0.0:14560')
while True:
    msg = m.recv_match(type='RC_CHANNELS', blocking=True)
    print(msg.chan1_raw, msg.chan3_raw)
