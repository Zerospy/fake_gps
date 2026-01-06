from pymavlink import mavutil
import time
import math


# Recibir MAVLink que RPanion te reenvía
in_mav = mavutil.mavlink_connection('udpin:0.0.0.0:14560')

# Enviar GPS_INPUT al bus MAVLink de RPanion
out_mav = mavutil.mavlink_connection('udpout:127.0.0.1:14550')

print("fake_gps: RX on 14560, TX to 14550")

# (Opcional) espera heartbeat desde el stream que llega por 14560
in_mav.wait_heartbeat(timeout=10)

# Posición inicial (puede ser cualquiera) -33.031123, -71.621050
lat = -33.03112      # Santiago ejemplo
lon = -71.62105
alt = 10

speed = 0.00001     # Ajusta sensibilidad
heading = 0

while True:
    msg = in_mav.recv_match(type='RC_CHANNELS', blocking=False)
    if not msg:
        continue

    # Canal típico: throttle y yaw
    throttle = msg.chan3_raw
    yaw = msg.chan4_raw

    v = (throttle - 1500) * speed
    heading += (yaw - 1500) * 0.01

    lat += v * math.cos(heading)
    lon += v * math.sin(heading)

    # Enviar por el socket de salida (out_mav). El objeto devuelto por
    # mavutil.mavlink_connection no expone métodos directos como
    # `gps_input_send`, hay que usar la propiedad `mav`.
    try:
        # Preparar campos en el orden correcto según GPS_INPUT (mavlink):
        # (time_usec, gps_id, ignore_flags, time_week_ms, time_week, fix_type,
        #  lat, lon, alt, hdop, vdop, vn, ve, vd,
        #  speed_accuracy, horiz_accuracy, vert_accuracy, satellites_visible, yaw)

        time_usec = int(time.time() * 1e6)
        gps_id = 0
        ignore_flags = 0
        time_week_ms = 0
        time_week = 0
        fix_type = 3  # 3D fix

        # Lat/Lon deben ir en int32 degE7
        lat_e7 = int(lat * 1e7)
        lon_e7 = int(lon * 1e7)

        # Alt es float (m)
        hdop = 0.1
        vdop = 0.1

        # Velocidades en m/s (north, east, down). Aquí aproximamos 0.
        vn = 0.0
        ve = 0.0
        vd = 0.0

        speed_accuracy = 0.1
        horiz_accuracy = 1.0
        vert_accuracy = 1.0

        satellites_visible = int(max(0, min(255, 10)))

        # Yaw en centigrados (cdeg, uint16): 0..36000
        yaw_cdeg = int((heading * 180.0 / math.pi) * 100) % 36000

        print(f"Enviando GPS_INPUT: lat={lat_e7}, lon={lon_e7}, alt={alt}, sats={satellites_visible}, yaw={yaw_cdeg}")

        out_mav.mav.gps_input_send(
            time_usec,
            gps_id,
            ignore_flags,
            time_week_ms,
            time_week,
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
    except Exception as e:
        # Evitar que el script termine si el envio falla; imprimir el error
        # para diagnóstico.
        print(f"Error enviando GPS_INPUT: {e}")

    time.sleep(0.2)
