from pymavlink import mavutil
import time
import math

# Conexión MAVLink (ajusta si usas otro puerto)
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()

# Posición inicial (puede ser cualquiera)
lat = -33.0472      # Santiago ejemplo
lon = -71.6127
alt = 10

speed = 0.00001     # Ajusta sensibilidad
heading = 0

while True:
    msg = master.recv_match(type='RC_CHANNELS', blocking=True)
    if not msg:
        continue

    # Canal típico: throttle y yaw
    throttle = msg.chan3_raw
    yaw = msg.chan4_raw

    v = (throttle - 1500) * speed
    heading += (yaw - 1500) * 0.01

    lat += v * math.cos(heading)
    lon += v * math.sin(heading)

    master.mav.gps_input_send(
        int(time.time() * 1e6),
        0,
        0,
        0,
        0,
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        1.0, 1.0,
        0.1, 0.1,
        0,
        10
    )

    time.sleep(0.2)
