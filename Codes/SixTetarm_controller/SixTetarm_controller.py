#!/usr/bin/env python3

# ============================================================
# CONTROLLER WEBOTS - SixTetarm
# Versione: Webots R2025a + ROS 2 Jazzy
# Funzione: muove il UR3e simulato alla posa X tramite tastiera
#           e pubblica /joint_states su ROS 2
# ============================================================

from controller import Robot, Keyboard
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


MOTOR_NAMES = [
    'shoulder_pan_joint',   # J1 - Base
    'shoulder_lift_joint',  # J2 - Spalla
    'elbow_joint',          # J3 - Gomito
    'wrist_1_joint',        # J4 - Polso Pitch
    'wrist_2_joint',        # J5 - Polso Roll
    'wrist_3_joint',        # J6 - solo sim
]


def deg(d):
    return math.radians(d)


# ============================================================
# SEZIONE 2: POSE
# POSE_X — "Pronto": braccio in posizione operativa
# ⚠️ Sostituisci questi valori con quelli MISURATI dal robot fisico
# Formula: rad = steps / steps_per_giro * 2π
# ============================================================
POSE_X = [
    deg(-20),    # J1 Base — dritto
    deg(0),   # J2 Spalla — 
    deg(20),   # J3 Gomito — sicuro sotto il limite fisico
    deg(90),   # J4 Polso Pitch 
    deg(0),    # J5 Polso Roll — neutro
    deg(0),    # J6 solo sim
]


# ============================================================
# POSA HOME — posizione di riposo (braccio compatto sul tavolo)
# Da usare come posizione iniziale sicura
# ⚠️ MISURA questi valori con il test motori prima di usarli!
# ============================================================
POSE_HOME = [
    deg(180),    # J1 Base
    deg(0),    # J2 Spalla — zero fisico = braccio compatto
    deg(0),    # J3 Gomito — zero fisico
    deg(0),    # J4 Polso Pitch
    deg(0),    # J5 Polso Roll
    deg(0),    # J6 solo sim
]


TIME_STEP = 32   # ms
VELOCITY  = 0.5  # rad/sec


class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('webots_joint_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info('Publisher /joint_states avviato')

    def publish(self, names, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = names
        msg.position = list(positions)
        self.publisher.publish(msg)


def main():

    rclpy.init()
    ros_node = JointStatePublisher()

    robot    = Robot()
    keyboard = Keyboard()
    keyboard.enable(TIME_STEP)

    motors  = [robot.getDevice(name) for name in MOTOR_NAMES]
    sensors = []

    for i, motor in enumerate(motors):
        if motor is None:
            print(f'[ERRORE] Motore non trovato: {MOTOR_NAMES[i]}')
        else:
            motor.setVelocity(VELOCITY)
            sensor = motor.getPositionSensor()
            sensor.enable(TIME_STEP)
            sensors.append(sensor)

    # Nomi attivi (solo motori trovati)
    active_names = [MOTOR_NAMES[i] for i, m in enumerate(motors) if m is not None]

    last_published = None
    current_pose   = 'HOME'

    # Porta il braccio in HOME all'avvio
    for i, motor in enumerate(motors):
        if motor is not None:
            motor.setPosition(POSE_HOME[i])

    print('============================================')
    print('Controller SixTetarm avviato!')
    print('  Premi X → Posa operativa (Pronto)')
    print('  Premi H → Posa HOME (riposo)')
    print('  Premi Q → Esci')
    print('============================================')

    while robot.step(TIME_STEP) != -1:

        key = keyboard.getKey()

        if key in (ord('X'), ord('x')):
            current_pose = 'X'
            print('[TASTO X] → Posa operativa')
            for i, motor in enumerate(motors):
                if motor is not None:
                    motor.setPosition(POSE_X[i])

        elif key in (ord('H'), ord('h')):
            current_pose = 'HOME'
            print('[TASTO H] → Posa HOME (riposo)')
            for i, motor in enumerate(motors):
                if motor is not None:
                    motor.setPosition(POSE_HOME[i])

        elif key in (ord('Q'), ord('q')):
            print('Uscita richiesta.')
            break

        current_positions = [s.getValue() for s in sensors]

        if last_published is None or any(
            abs(current_positions[i] - last_published[i]) > 0.01
            for i in range(len(current_positions))
        ):
            ros_node.publish(active_names, current_positions)
            last_published = list(current_positions)

        rclpy.spin_once(ros_node, timeout_sec=0)

    ros_node.destroy_node()
    rclpy.shutdown()
    print('Controller fermato.')


if __name__ == '__main__':
    main()