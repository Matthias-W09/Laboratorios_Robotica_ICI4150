"""lab2controller_epuck controller."""
from controller import Robot
import math

TIME_STEP = 32                  # ms
TS = TIME_STEP / 1000.0         # segundos
FS = 1.0 / TS                   # Hz

WHEEL_RADIUS = 0.0205           # metros	
MAX_SPEED = 4.5                 # rad/s

UMBRAL_ACTIVAR = 0.22           # metros
UMBRAL_DESACTIVAR = 0.25        # metros

# Sensores laterales
LATERAL_ALERT = 150             

FILTER_WIN = 5

# KALMAN

Q = 0.001
R = 0.05

P0 = 0.1
D0 = 0.40

# CORRECCIÓN LATERAL
K_LATERAL = 0.0005

en_evasion = False
direccion_giro = 0

def ir_to_meters(val):

    if val <= 50.0:
        return 0.40

    elif val <= 150.0:
        return 0.30 - (val - 50.0) * (0.15 / 100.0)

    elif val <= 800.0:
        return 0.15 - (val - 150.0) * (0.09 / 650.0)

    elif val <= 2500.0:
        return 0.06 - (val - 800.0) * (0.03 / 1700.0)

    else:
        return 0.02

# FILTRO MEDIA MÓVIL

def moving_average(buffer, value, window):

    buffer.append(value)

    if len(buffer) > window:
        buffer.pop(0)

    return sum(buffer) / len(buffer)


# INICIALIZACIÓN ROBOT

robot = Robot()

# MOTORES

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# SENSORES

ps = []

for i in range(8):

    sensor = robot.getDevice(f'ps{i}')
    sensor.enable(TIME_STEP)

    ps.append(sensor)

# ENCODERS

left_encoder = robot.getDevice('left wheel sensor')
right_encoder = robot.getDevice('right wheel sensor')

left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)


# VARIABLES

prev_left_enc = 0.0
prev_right_enc = 0.0

first_step = True

# Buffers filtros
front_buffer = []

left_lat_buffer = []
right_lat_buffer = []

# Estado Kalman
d_est = D0
P_est = P0

# LOGS

raw_front_log = []
filtered_front_log = []
kalman_front_log = []

left_enc_log = []
right_enc_log = []

time_log = []

step_count = 0

# INFORMACION

print("================================================")
print("CONTROLADOR AVANZADO ACTIVO")
print(f"Ts = {TS:.3f} s")
print(f"Fs = {FS:.2f} Hz")
print("================================================")

while robot.step(TIME_STEP) != -1:

    # 1. LECTURA SENSORES

    front_left_val = ps[0].getValue()
    front_right_val = ps[7].getValue()

    raw_front = (
        front_left_val + front_right_val
    ) / 2.0

    # Sensores diagonales/laterales
    lateral_left_raw = ps[6].getValue()
    lateral_right_raw = ps[1].getValue()

    # 2. FILTRADO SIMPLE

    filtered_front = moving_average(
        front_buffer,
        raw_front,
        FILTER_WIN
    )

    lateral_left = moving_average(
        left_lat_buffer,
        lateral_left_raw,
        FILTER_WIN
    )

    lateral_right = moving_average(
        right_lat_buffer,
        lateral_right_raw,
        FILTER_WIN
    )

    # Conversión a metros
    z_k = ir_to_meters(filtered_front)

    # 3. ENCODERS

    cur_left_enc = left_encoder.getValue()
    cur_right_enc = right_encoder.getValue()

    if first_step:

        prev_left_enc = cur_left_enc
        prev_right_enc = cur_right_enc

        first_step = False

    # 4. ODOMETRÍA

    delta_left = (
        (cur_left_enc - prev_left_enc)
        * WHEEL_RADIUS
    )

    delta_right = (
        (cur_right_enc - prev_right_enc)
        * WHEEL_RADIUS
    )

    # Avance promedio
    delta_d = (
        delta_left + delta_right
    ) / 2.0

    prev_left_enc = cur_left_enc
    prev_right_enc = cur_right_enc

    # 5. FILTRO DE KALMAN

    # PREDICCIÓN

    d_pred = d_est - delta_d

    P_pred = P_est + Q

    # CORRECCIÓN

    K = P_pred / (P_pred + R)

    d_est = d_pred + K * (z_k - d_pred)

    P_est = (1.0 - K) * P_pred

    # Evitar negativos
    if d_est < 0.0:
        d_est = 0.0

    # 6. NAVEGACIÓN REACTIVA

    if not en_evasion:

        # OBSTÁCULO FRONTAL

        if d_est < UMBRAL_ACTIVAR:

            en_evasion = True

            # Decide hacia dónde girar
            if lateral_left > lateral_right:
                direccion_giro = 1
            else:
                direccion_giro = -1

            print(
                f"[OBSTÁCULO FRONTAL] "
                f"Kalman={d_est:.3f} m"
            )

        else:

            # EVITACIÓN LATERAL INTELIGENTE

            left_speed = MAX_SPEED
            right_speed = MAX_SPEED

            # Pared diagonal izquierda

            if lateral_left > LATERAL_ALERT:

                left_speed = MAX_SPEED * 0.9
                right_speed = MAX_SPEED * 0.3

                print(
                    f"[CORRIGIENDO DERECHA] "
                    f"L={lateral_left:.1f}"
                )

            # Pared diagonal derecha

            elif lateral_right > LATERAL_ALERT:

                left_speed = MAX_SPEED * 0.3
                right_speed = MAX_SPEED * 0.9

                print(
                    f"[CORRIGIENDO IZQUIERDA] "
                    f"R={lateral_right:.1f}"
                )

            # Corrección suave centrada

            else:

                error_lateral = (
                    lateral_left - lateral_right
                )

                correction = (
                    error_lateral * K_LATERAL
                )

                left_speed = (
                    MAX_SPEED - correction
                )

                right_speed = (
                    MAX_SPEED + correction
                )

            # Saturación
            left_speed = max(
                min(left_speed, MAX_SPEED),
                -MAX_SPEED
            )

            right_speed = max(
                min(right_speed, MAX_SPEED),
                -MAX_SPEED
            )

            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)

            # Debug
            if step_count % 15 == 0:

                print(
                    f"[AVANZANDO] "
                    f"K={d_est:.3f}m | "
                    f"Crudo={raw_front:.1f} | "
                    f"Filtrado={filtered_front:.1f} | "
                    f"L={lateral_left:.1f} | "
                    f"R={lateral_right:.1f}"
                )

    else:

        # MODO EVASIÓN

        if d_est > UMBRAL_DESACTIVAR:

            en_evasion = False
            direccion_giro = 0

            print(
                f"[CAMINO DESPEJADO] "
                f"Distancia={d_est:.3f} m"
            )

        # Giro evasivo
        if direccion_giro == 1:

            left_motor.setVelocity(
                MAX_SPEED * 0.5
            )

            right_motor.setVelocity(
                -MAX_SPEED * 0.5
            )

        else:

            left_motor.setVelocity(
                -MAX_SPEED * 0.5
            )

            right_motor.setVelocity(
                MAX_SPEED * 0.5
            )

    # 7. LOGS

    raw_front_log.append(raw_front)

    filtered_front_log.append(filtered_front)

    kalman_front_log.append(d_est)

    left_enc_log.append(cur_left_enc)
    right_enc_log.append(cur_right_enc)

    time_log.append(step_count * TS)

    step_count += 1
