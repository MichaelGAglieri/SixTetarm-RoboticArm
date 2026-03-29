#!/usr/bin/env python3

# ============================================================
# ROS 2 BRIDGE NODE - Braccio Robotico
# Legge joint_states da Webots e manda step ad Arduino via USB
# ============================================================

import rclpy                          # libreria principale ROS 2
from rclpy.node import Node           # classe base per ogni nodo ROS 2
from sensor_msgs.msg import JointState  # tipo di messaggio per gli angoli
import serial                         # libreria per comunicazione USB seriale
import serial.tools.list_ports        # per trovare la porta Arduino
import math                           # per convertire rad → gradi

# ============================================================
# COSTANTI - devono corrispondere esattamente al firmware Arduino
# ============================================================
MICROSTEP        = 8
STEP_PER_REV     = 200
STEP_PER_REV_MS  = STEP_PER_REV * MICROSTEP          # 1600 step/giro motore
GEAR_RATIO       = 38.4
STEPS_OUTPUT_REV = int(STEP_PER_REV_MS * GEAR_RATIO)  # 61440 step/giro uscita
STEPS_PER_DEGREE = STEPS_OUTPUT_REV / 360.0           # 170.67 step/grado

# ============================================================
# CONFIGURAZIONE SERIALE
# Cambia SERIAL_PORT se Arduino non è su /dev/ttyACM0
# Puoi trovare la porta con: ls /dev/ttyACM* oppure dmesg | tail
# BAUD_RATE deve corrispondere al Serial.begin() di Arduino (115200)
# ============================================================
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE   = 115200

# ============================================================
# MAPPING GIUNTI
# Webots pubblica i joint_states con i nomi dei giunti del UR3e
# Questo dizionario mappa ogni nome → indice J1..J5 del nostro braccio
# ⚠️ Controlla i nomi esatti nel tuo mondo Webots con:
#    ros2 topic echo /joint_states
# e aggiorna i nomi qui se diversi
# ============================================================
JOINT_MAPPING = {
    'shoulder_pan_joint':   0,   # J1 - Base
    'shoulder_lift_joint':  1,   # J2 - Spalla
    'elbow_joint':          2,   # J3 - Gomito
    'wrist_1_joint':        3,   # J4 - Polso Pitch
    'wrist_2_joint':        4,   # J5 - Polso Roll
    # 'wrist_3_joint' ignorato: non esiste nel braccio reale
}

# ============================================================
# CLASSE BRIDGE NODE
# Eredita da Node: è il nodo ROS 2 vero e proprio
# ============================================================
class RobotBridgeNode(Node):

    def __init__(self):
        # Inizializza il nodo con il nome 'robot_bridge'
        # Questo nome appare quando fai: ros2 node list
        super().__init__('robot_bridge')
        self.get_logger().info('Bridge node avviato...')

        # --- Connessione seriale ad Arduino ---
        # Tenta la connessione, se fallisce stampa un errore chiaro
        try:
            self.serial_conn = serial.Serial(
                SERIAL_PORT,
                BAUD_RATE,
                timeout=1  # aspetta max 1 secondo per risposta Arduino
            )
            self.get_logger().info(f'Arduino connesso su {SERIAL_PORT} a {BAUD_RATE} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'ERRORE connessione Arduino: {e}')
            self.get_logger().error('Controlla che Arduino sia collegato e la porta sia corretta')
            self.serial_conn = None

        # --- Subscriber al topic /joint_states ---
        # Ogni volta che Webots pubblica nuovi angoli, viene chiamata
        # automaticamente la funzione joint_states_callback()
        self.subscription = self.create_subscription(
            JointState,           # tipo di messaggio atteso
            '/joint_states',      # nome del topic da ascoltare
            self.joint_states_callback,  # funzione da chiamare ad ogni messaggio
            10                    # dimensione coda messaggi (tieni gli ultimi 10)
        )

        # Array che memorizza la posizione corrente in step per ogni giunto
        # Inizializzato a 0 = posizione HOME
        self.current_steps = [0, 0, 0, 0, 0]

        # Soglia minima di movimento: ignora variazioni sotto X step
        # Evita di mandare comandi ad Arduino per microvariazioni del simulatore
        self.MIN_STEP_CHANGE = 10  # step (= ~0.06°)

        self.get_logger().info('In ascolto su /joint_states...')

    # ============================================================
    # CALLBACK - chiamata ogni volta che Webots manda nuovi angoli
    # msg è un oggetto JointState con:
    #   msg.name[]     → lista nomi giunti  (es. ['shoulder_pan_joint', ...])
    #   msg.position[] → lista angoli [rad] (es. [1.57, 0.78, ...])
    # ============================================================
    def joint_states_callback(self, msg):

        # Array temporaneo per i nuovi target in step
        new_steps = list(self.current_steps)  # copia posizione attuale
        changed   = False  # flag: c'è stato almeno un cambiamento significativo?

        # Itera su ogni giunto pubblicato da Webots
        for i, joint_name in enumerate(msg.name):

            # Controlla se questo giunto è nel nostro mapping
            # (ignora giunto 6 e pinza che non esistono nel reale)
            if joint_name not in JOINT_MAPPING:
                continue  # salta questo giunto

            joint_idx = JOINT_MAPPING[joint_name]  # indice 0-4

            # Prende l'angolo in radianti per questo giunto
            angle_rad = msg.position[i]

            # Converte: radianti → gradi → step
            angle_deg  = math.degrees(angle_rad)          # rad → gradi
            target_step = int(angle_deg * STEPS_PER_DEGREE)  # gradi → step

            # Controlla se la variazione è abbastanza grande da giustificare
            # un nuovo comando ad Arduino (filtra il rumore del simulatore)
            delta = abs(target_step - self.current_steps[joint_idx])
            if delta >= self.MIN_STEP_CHANGE:
                new_steps[joint_idx] = target_step
                changed = True  # segna che almeno un giunto è cambiato

        # Manda il comando ad Arduino solo se qualcosa è cambiato
        # e la connessione seriale è attiva
        if changed and self.serial_conn is not None:
            self.send_to_arduino(new_steps)
            self.current_steps = new_steps  # aggiorna stato corrente

    # ============================================================
    # FUNZIONE: send_to_arduino()
    # Costruisce la stringa di comando e la manda via USB
    # Formato: "J1:500 J2:-200 J3:1500 J4:800 J5:300\n"
    # ============================================================
    def send_to_arduino(self, steps):

        # Costruisce la stringa con tutti e 5 i giunti
        command = (
            f"J1:{steps[0]} "
            f"J2:{steps[1]} "
            f"J3:{steps[2]} "
            f"J4:{steps[3]} "
            f"J5:{steps[4]}\n"  # \n = newline, segnale di fine comando per Arduino
        )

        try:
            # Encode converte la stringa Python in bytes (necessario per seriale)
            self.serial_conn.write(command.encode('utf-8'))
            self.get_logger().info(f'Inviato: {command.strip()}')

            # Legge eventuali risposte di debug da Arduino (opzionale)
            if self.serial_conn.in_waiting > 0:
                response = self.serial_conn.readline().decode('utf-8').strip()
                self.get_logger().info(f'Arduino risponde: {response}')

        except serial.SerialException as e:
            self.get_logger().error(f'Errore invio seriale: {e}')

    # ============================================================
    # FUNZIONE: destroy_node()
    # Chiamata automaticamente quando il nodo viene fermato (Ctrl+C)
    # Chiude la connessione seriale in modo pulito
    # ============================================================
    def destroy_node(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info('Connessione seriale chiusa.')
        super().destroy_node()


# ============================================================
# MAIN - punto di ingresso del programma
# ============================================================
def main(args=None):
    # Inizializza il sistema ROS 2
    rclpy.init(args=args)

    # Crea il nodo bridge
    node = RobotBridgeNode()

    try:
        # spin() mantiene il nodo in ascolto finché non premi Ctrl+C
        # Senza spin() il nodo si avvierebbe e si chiuderebbe subito
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo fermato con Ctrl+C')
    finally:
        # Pulizia finale: chiude seriale e spegne ROS 2
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""

Vedi tutti i nodi attivi (deve apparire /robot_bridge)

ros2 node list

Vedi tutti i topic attivi

ros2 topic list

Spia i messaggi che arrivano da Webots in tempo reale

ros2 topic echo /joint_states

Manda un comando di test manuale senza Webots

ros2 topic pub /joint_states sensor_msgs/JointState \
  "name: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint']
   
   position: [0.785, 0.523, -0.785, 0.523, 0.0]
   
   """
