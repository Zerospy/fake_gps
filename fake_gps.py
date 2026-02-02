from pymavlink import mavutil
import os
import time
import math

# RX: RC_CHANNELS (desde FC via rpanion/router)
IN_UDP_PORT = int(os.getenv("IN_UDP_PORT", "14560"))
IN_MAVLINK = os.getenv("IN_MAVLINK")
in_conn = IN_MAVLINK or f'udpin:0.0.0.0:{IN_UDP_PORT}'
in_mav = mavutil.mavlink_connection(in_conn)

# TX: hacia el bus MAVLink que llega al FC (si corres esto en la RPi, localhost ok)
OUT_UDP_HOST = os.getenv("OUT_UDP_HOST", "10.0.2.100")
OUT_UDP_PORT = int(os.getenv("OUT_UDP_PORT", "14551"))
OUT_MAVLINK = os.getenv("OUT_MAVLINK")
out_conn = OUT_MAVLINK or f'udpout:{OUT_UDP_HOST}:{OUT_UDP_PORT}'
out_mav = mavutil.mavlink_connection(out_conn)

# Rover/USV: puede ser timón+hélice ("steer") o empuje diferencial ("diff").
DRIVE_MODE = os.getenv("DRIVE_MODE", "steer").strip().lower()  # steer|diff
STEER_SERVO_CH = int(os.getenv("STEER_SERVO_CH", "1"))
THROTTLE_SERVO_CH = int(os.getenv("THROTTLE_SERVO_CH", "3"))
LEFT_THROTTLE_SERVO_CH = int(os.getenv("LEFT_THROTTLE_SERVO_CH", "1"))
RIGHT_THROTTLE_SERVO_CH = int(os.getenv("RIGHT_THROTTLE_SERVO_CH", "3"))

print(f"fake_gps: RX={in_conn}, TX={out_conn}, DRIVE_MODE={DRIVE_MODE}")
if DRIVE_MODE == "steer":
    print(f"fake_gps: steer_ch=servo{STEER_SERVO_CH} throttle_ch=servo{THROTTLE_SERVO_CH}")
elif DRIVE_MODE == "diff":
    print(f"fake_gps: left_throttle_ch=servo{LEFT_THROTTLE_SERVO_CH} right_throttle_ch=servo{RIGHT_THROTTLE_SERVO_CH}")
else:
    print("WARN: unknown DRIVE_MODE; expected 'steer' or 'diff'")

hb0 = in_mav.wait_heartbeat(timeout=10)

if not hb0:
    print("WARN: no HEARTBEAT after 10s; continuing without mode info")

    
# Posición inicial (Valparaíso aprox)
lat = -33.03112
lon = -71.62105
alt = 10.0

# Estado
heading_rad = 0.0  # rad
last_t = time.time()
last_rc_time = None
last_throttle = 1500
last_yaw = 1500
last_servo_time = None
last_throttle_out = 1500
last_yaw_out = 1500
last_left_out = 1500
last_right_out = 1500

# Ajustes (tú los calibras)
MAX_SPEED_MPS = 3.0        # velocidad máxima (m/s) con throttle al 2000
MAX_TURN_RATE_DPS = 45.0   # giro máximo (deg/s) con yaw al 2000  
RC_STALE_S = 1.0           # si no hay RC reciente, usa neutral
SERVO_STALE_S = 0.5        # si no hay SERVO_OUTPUT reciente, cae a RC

R_EARTH = 6378137.0  # m

SEND_GPS_YAW = os.getenv("SEND_GPS_YAW", "0").strip() in {"1", "true", "TRUE", "yes", "YES"}
TURN_RATE_SCALES_WITH_SPEED = os.getenv("TURN_RATE_SCALES_WITH_SPEED", "1").strip() in {"1", "true", "TRUE", "yes", "YES"}

def clamp(x, a, b):
    return max(a, min(b, x))


current_mode = None

GPS_EPOCH_UNIX_S = 315964800  # 1980-01-06
GPS_WEEK_S = 7 * 24 * 60 * 60
# GPS-UTC offset. Since 2017-01-01 it's 18s; if this changes, update this constant.
GPS_UTC_LEAP_S = 18

def unix_to_gps_week_and_ms(unix_s: float) -> tuple[int, int]:
    gps_s = unix_s - GPS_EPOCH_UNIX_S + GPS_UTC_LEAP_S
    if gps_s < 0:
        gps_s = 0
    week = int(gps_s // GPS_WEEK_S)
    week_ms = int((gps_s - (week * GPS_WEEK_S)) * 1000.0)
    return week, week_ms


while True:
    now = time.time()
    dt = now - last_t
    last_t = now
    if dt <= 0:
        dt = 0.02
        
    # Lee y procesa varios mensajes por tick para no "perder" streams por orden de llegada.
    for _ in range(20):
        m = in_mav.recv_match(blocking=False)
        if not m:
            break
        mtype = m.get_type()
        if mtype == 'HEARTBEAT':
            current_mode = mavutil.mode_string_v10(m)
        elif mtype == 'RC_CHANNELS':
            last_throttle = m.chan3_raw  # 1000-2000 típico
            last_yaw = m.chan4_raw       # 1000-2000 típico
            last_rc_time = now
        elif mtype == 'SERVO_OUTPUT_RAW':
            # Usa los outputs del autopiloto para que en GUIDED/AUTO el "fake GPS" reaccione a waypoints.
            if DRIVE_MODE == "diff":
                left = getattr(m, f'servo{LEFT_THROTTLE_SERVO_CH}_raw', None)
                right = getattr(m, f'servo{RIGHT_THROTTLE_SERVO_CH}_raw', None)
                if left is not None:
                    last_left_out = int(left)
                if right is not None:
                    last_right_out = int(right)
            else:
                steer = getattr(m, f'servo{STEER_SERVO_CH}_raw', None)
                thr = getattr(m, f'servo{THROTTLE_SERVO_CH}_raw', None)
                if steer is not None:
                    last_yaw_out = int(steer)
                if thr is not None:
                    last_throttle_out = int(thr)
            last_servo_time = now

    # Selecciona la "entrada" para integrar el movimiento:
    # - En GUIDED/AUTO: preferir SERVO_OUTPUT_RAW (lo que el autopiloto manda) para que siga waypoints.
    # - En MANUAL/otros: usar RC.
    use_servo = (
        current_mode in {"GUIDED", "AUTO"}
        and last_servo_time
        and (now - last_servo_time) <= SERVO_STALE_S
    )

    if use_servo:
        if DRIVE_MODE == "diff":
            throttle = int((last_left_out + last_right_out) / 2)
            # Yaw command from differential thrust (right-left); maps to [-1..+1] around neutral.
            yaw = int(1500 + (last_right_out - last_left_out) / 2)
        else:
            throttle = last_throttle_out
            yaw = last_yaw_out
    else:
        if not last_rc_time or (now - last_rc_time) > RC_STALE_S:
            throttle = 1500
            yaw = 1500
        else:
            throttle = last_throttle
            yaw = last_yaw

    # Normaliza [-1..+1]
    thr_n = clamp((throttle - 1500) / 500.0, -1.0, 1.0)
    yaw_n = clamp((yaw - 1500) / 500.0, -1.0, 1.0)

    # Velocidad y giro
    # En Rover/USV, forzar velocidad fija en GUIDED suele disparar "pos horiz variance" (EKF ve
    # movimiento sin aceleración/IMU real). Mantén la velocidad ligada al throttle con deadband.
    thr_deadband = 0.05
    if abs(thr_n) < thr_deadband:
        thr_n_eff = 0.0
    else:
        thr_n_eff = thr_n

    speed_mps = thr_n_eff * MAX_SPEED_MPS

    # Para Rover/USV el "steering" no es una tasa de giro directa; depende de la velocidad.
    # Si no escalas con velocidad, el modelo tiende a girar demasiado y puede terminar en círculos.
    if TURN_RATE_SCALES_WITH_SPEED and MAX_SPEED_MPS > 1e-6:
        speed_scale = min(1.0, abs(speed_mps) / MAX_SPEED_MPS)
    else:
        speed_scale = 1.0

    turn_rate_rps = math.radians(yaw_n * MAX_TURN_RATE_DPS) * speed_scale

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

    # Yaw en cdeg [0..36000]. En banco, el compás/IMU no cambia de yaw, así que GUIDED puede quedarse
    # "tratando de girar" para siempre. Si quieres que el Rover use el yaw del GPS, activa SEND_GPS_YAW=1
    # y configura el FC para aceptar yaw externo.
    if SEND_GPS_YAW:
        yaw_cdeg = int((math.degrees(heading_rad) * 100.0)) % 36000
    else:
        yaw_cdeg = 65535

    # Si quieres, puedes ignorar campos que no te importen con flags.
    # Aquí mandamos todo coherente, así que 0.
    # With fake GPS, avoid advertising unrealistically good accuracy/HDOP.
    ignore_flags = (
        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_HDOP
        | mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VDOP
        | mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY
        | mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY
        | mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY
    )

    time_week, time_week_ms = unix_to_gps_week_and_ms(now)

    out_mav.mav.gps_input_send(
        time_usec,
        gps_id,
        ignore_flags,
        int(time_week_ms),
        int(time_week),
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
