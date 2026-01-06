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
lat = -33.031123      # Santiago ejemplo
lon = -71.621050
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
        out_mav.mav.gps_input_send(
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
    except Exception as e:
        # Evitar que el script termine si el envio falla; imprimir el error
        # para diagnóstico.
        print(f"Error enviando GPS_INPUT: {e}")

    time.sleep(0.2)
