from pymavlink import mavutil
import time
import math

# RX: RC_CHANNELS (desde FC via rpanion/router)
in_mav = mavutil.mavlink_connection('tcpin:0.0.0.0:14560')

# TX: hacia el bus MAVLink que llega al FC (si corres esto en la RPi, localhost ok)
out_mav = mavutil.mavlink_connection('tcpout:127.0.0.1:14550')

print("fake_gps: RX on 14560, TX to 14550")

in_mav.wait_heartbeat(timeout=10)

# Posición inicial (Valparaíso aprox)
lat = -33.03112
lon = -71.62105
alt = 10.0

# Estado
heading_rad = 0.0  # rad
last_t = time.time()

# Ajustes (tú los calibras)
MAX_SPEED_MPS = 3.0        # velocidad máxima (m/s) con throttle al 2000
MAX_TURN_RATE_DPS = 45.0   # giro máximo (deg/s) con yaw al 2000

R_EARTH = 6378137.0  # m

def clamp(x, a, b):
    return max(a, min(b, x))

while True:
    now = time.time()
    dt = now - last_t
    last_t = now
    if dt <= 0:
        dt = 0.02

    msg = in_mav.recv_match(type='RC_CHANNELS', blocking=False)
    if not msg:
        time.sleep(0.02)
        continue

    throttle = msg.chan3_raw  # 1000-2000 típico
    yaw = msg.chan4_raw       # 1000-2000 típico

    # Normaliza [-1..+1]
    thr_n = clamp((throttle - 1500) / 500.0, -1.0, 1.0)
    yaw_n = clamp((yaw - 1500) / 500.0, -1.0, 1.0)

    # Velocidad y giro
    speed_mps = thr_n * MAX_SPEED_MPS
    turn_rate_rps = math.radians(yaw_n * MAX_TURN_RATE_DPS)

    # Actualiza heading
    heading_rad = (heading_rad + turn_rate_rps * dt) % (2 * math.pi)

    # Velocidad en N/E (m/s)
    vn = speed_mps * math.cos(heading_rad)  # north
    ve = speed_mps * math.sin(heading_rad)  # east
    vd = 0.0

    # Integra posición (m -> deg)
    dlat = (vn * dt / R_EARTH) * (180.0 / math.pi)
    dlon = (ve * dt / (R_EARTH * math.cos(math.radians(lat)))) * (180.0 / math.pi)

    lat += dlat
    lon += dlon

    # MAVLink GPS_INPUT
    time_usec = int(now * 1e6)
    gps_id = 0
    fix_type = 3  # 3D fix

    lat_e7 = int(lat * 1e7)
    lon_e7 = int(lon * 1e7)

    # Precisión/HDOP razonable
    hdop = 0.8
    vdop = 1.2

    speed_accuracy = 0.5
    horiz_accuracy = 1.5
    vert_accuracy = 2.5

    satellites_visible = 12

    # Yaw en cdeg [0..36000]
    yaw_cdeg = int((math.degrees(heading_rad) * 100.0)) % 36000

    # Si quieres, puedes ignorar campos que no te importen con flags.
    # Aquí mandamos todo coherente, así que 0.
    ignore_flags = 0

    out_mav.mav.gps_input_send(
        time_usec,
        gps_id,
        ignore_flags,
        0,  # time_week_ms (opcional)
        0,  # time_week (opcional)
        fix_type,
        lat_e7,
        lon_e7,
        float(alt),
        float(hdop),
        float(vdop),
        float(vn),
        float(ve),
        float(vd),
        float(speed_accuracy),
        float(horiz_accuracy),
        float(vert_accuracy),
        int(satellites_visible),
        int(yaw_cdeg),
    )

    print(f"GPS_INPUT lat={lat:.6f} lon={lon:.6f} spd={speed_mps:.2f}m/s hdg={math.degrees(heading_rad):.1f}")
    time.sleep(0.1)
