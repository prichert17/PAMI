import serial
import threading
import re
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
import sys
import time
from collections import deque

# ----- CONFIG -----
BLUETOOTH_PORT = 'COM8'      # adapte ici !
BAUDRATE = 115200
TIMEOUT = 1

# ----- Variables partagées -----
terrain_str = "Inconnu"
robot_pos = [0, 0]
target_pos = [0, 0]
robot_history = []
target_history = []
lock = threading.Lock()
connection_ok = threading.Event()
RESET_DIST = 50  # distance seuil (en cm) pour détecter un "reset"
start_time = None  # Timer de démarrage
last_robot_move_time = None
robot_stopped = False
ser = None  # Variable globale pour la connexion série
message_log = deque(maxlen=25)  # Log des 25 derniers messages

def parse_line(line):
    global terrain_str, robot_pos, target_pos, start_time, last_robot_move_time, robot_stopped
    
    # Ajouter le message au log
    with lock:
        message_log.append(line[:60])  # Limite à 60 caractères
    
    # Terrain color
    m = re.search(r'Terrain (\w+)', line, re.IGNORECASE)
    if m:
        c = m.group(1).lower()
        with lock:
            if c == "jaune":
                terrain_str = "Jaune"
            elif c == "bleu":
                terrain_str = "Bleu"
            else:
                terrain_str = "Inconnu"
    # Target
    m = re.search(r'TargetX\s*:\s*([\d\.]+).*TargetY\s*:\s*([\d\.]+)', line)
    if m:
        with lock:
            target_pos[0] = float(m.group(1))
            target_pos[1] = float(m.group(2))
            target_history.append(tuple(target_pos))
    # Robot
    m = re.search(r'RobotX\s*:\s*([\d\.]+).*RobotY\s*:\s*([\d\.]+)', line)
    if m:
        with lock:
            new_x = float(m.group(1))
            new_y = float(m.group(2))
            # Démarre le timer si ce n'est pas déjà fait
            if start_time is None:
                start_time = time.time()
            # Détecte un reset si la distance > seuil
            if robot_history:
                last_x, last_y = robot_history[-1]
                dist = ((new_x - last_x) ** 2 + (new_y - last_y) ** 2) ** 0.5
                if dist > RESET_DIST:
                    robot_history.clear()
                    target_history.clear()
                    robot_pos[0] = 0
                    robot_pos[1] = 0
                    target_pos[0] = 0
                    target_pos[1] = 0
                    start_time = None
                    last_robot_move_time = None
                    robot_stopped = False
            # Détecte si le robot a bougé
            if not robot_history or (new_x, new_y) != robot_history[-1]:
                last_robot_move_time = time.time()
                robot_stopped = False
            robot_pos[0] = new_x
            robot_pos[1] = new_y
            robot_history.append((new_x, new_y))

def serial_thread():
    global ser
    try:
        ser = serial.Serial(BLUETOOTH_PORT, BAUDRATE, timeout=TIMEOUT)
        connection_ok.set()
    except Exception as e:
        print(f"ERREUR : Impossible d'ouvrir le port {BLUETOOTH_PORT} : {e}")
        connection_ok.clear()
        return

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                parse_line(line)
    except Exception as e:
        print("Erreur série :", e)
    finally:
        if ser:
            ser.close()

def reset_robot(event):
    """Fonction appelée quand on clique sur Reset"""
    global ser, robot_history, target_history, robot_pos, target_pos, start_time, last_robot_move_time, robot_stopped
    if ser and ser.is_open:
        try:
            ser.write(b'RESET\n')
            print("RESET envoyé")
            # Reset immédiat de l'interface
            with lock:
                robot_history.clear()
                target_history.clear()
                robot_pos[0] = 0
                robot_pos[1] = 0
                target_pos[0] = 0
                target_pos[1] = 0
                start_time = None
                last_robot_move_time = None
                robot_stopped = False
                message_log.append("=== RESET MANUEL ===")
        except Exception as e:
            print(f"Erreur envoi RESET: {e}")

def reconnect(event):
    """Fonction appelée quand on clique sur Connect - Réinitialise tout"""
    global ser, connection_ok, robot_history, target_history, robot_pos, target_pos, start_time, last_robot_move_time, robot_stopped
    try:
        # Ferme l'ancienne connexion
        if ser and ser.is_open:
            ser.close()
        
        # Réinitialise toutes les variables
        with lock:
            robot_history.clear()
            target_history.clear()
            robot_pos[0] = 0
            robot_pos[1] = 0
            target_pos[0] = 0
            target_pos[1] = 0
            start_time = None
            last_robot_move_time = None
            robot_stopped = False
            message_log.clear()
            message_log.append("=== RECONNEXION ===")
        
        # Nouvelle connexion
        ser = serial.Serial(BLUETOOTH_PORT, BAUDRATE, timeout=TIMEOUT)
        connection_ok.set()
        print("Reconnexion réussie - Interface réinitialisée")
        
    except Exception as e:
        print(f"Erreur reconnexion: {e}")
        connection_ok.clear()
        with lock:
            message_log.append(f"ERREUR: {str(e)[:40]}")

# ----- Thread série -----
t = threading.Thread(target=serial_thread, daemon=True)
t.start()
# Attend la connexion :
connection_ok.wait(timeout=3)

if not connection_ok.is_set():
    print(f"\nConnexion impossible sur {BLUETOOTH_PORT}. Aucune interface graphique n'est lancée.\n")
    sys.exit(1)

# ----- Interface graphique -----
fig = plt.figure(figsize=(18, 9))  # Plus large pour plus de place
gs = fig.add_gridspec(1, 2, width_ratios=[2.5, 1])

# Graphique principal
ax = fig.add_subplot(gs[0])
ax.set_title("PAMI : Position robot et cible")
ax.set_xlim(0, 200)
ax.set_ylim(0, 300)
ax.set_xlabel("Y (cm)")
ax.set_ylabel("X (cm)")

robot_dot, = ax.plot([], [], 'o', color='red', markersize=16, label="Robot (actuel)")
target_dot, = ax.plot([], [], '*', color='lime', markersize=20, label="Cible (actuelle)")
robot_hist_plot, = ax.plot([], [], '.', color='crimson', alpha=0.25, markersize=8, label="Trajet robot")
target_hist_plot, = ax.plot([], [], '.', color='deepskyblue', alpha=0.7, markersize=18, label="Historique cibles")

text_robot = ax.text(5, 285, "", fontsize=12, color="red", weight="bold")
text_target = ax.text(5, 272, "", fontsize=12, color="green", weight="bold")
text_terrain = ax.text(5, 259, "", fontsize=12, color="black", weight="bold")
text_timer = ax.text(120, 285, "", fontsize=12, color="blue", weight="bold")

ax.legend(loc="lower right")

# Zone des messages - Plus grande et avec plus de messages
ax_messages = fig.add_subplot(gs[1])
ax_messages.set_title("Messages reçus (30 derniers)", fontsize=11)
ax_messages.set_xlim(0, 1)
ax_messages.set_ylim(0, 1)
ax_messages.axis('off')

text_messages = ax_messages.text(0.02, 0.98, "", fontsize=9, color="black", 
                               family="monospace", verticalalignment='top', 
                               transform=ax_messages.transAxes)

# Boutons
ax_reset = plt.axes([0.02, 0.95, 0.06, 0.04])
ax_connect = plt.axes([0.09, 0.95, 0.08, 0.04])  # Un peu plus large
btn_reset = Button(ax_reset, 'RESET')
btn_connect = Button(ax_connect, 'Reconnect')  # Nom plus explicite
btn_reset.on_clicked(reset_robot)
btn_connect.on_clicked(reconnect)

plt.tight_layout()

def update(_):
    global robot_stopped
    with lock:
        rx, ry = robot_pos
        tx, ty = target_pos
        rob_hist = list(robot_history)
        tar_hist = list(target_history)
        terr = terrain_str
        st = start_time
        last_move = last_robot_move_time
        messages = list(message_log)
    
    # Axes inversés :
    robot_dot.set_data([ry], [rx])
    target_dot.set_data([ty], [tx])
    
    if rob_hist:
        ys, xs = zip(*rob_hist)
        robot_hist_plot.set_data(xs, ys)
    else:
        robot_hist_plot.set_data([], [])
        
    if tar_hist:
        ys, xs = zip(*tar_hist)
        target_hist_plot.set_data(xs, ys)
    else:
        target_hist_plot.set_data([], [])
    
    text_robot.set_text(f"Robot : ({rx:.1f}, {ry:.1f})")
    text_target.set_text(f"Cible : ({tx:.1f}, {ty:.1f})")
    text_terrain.set_text(f"Terrain : {terr}")
    
    # Affichage des messages (30 derniers, texte plus grand)
    if messages:
        # Limite chaque ligne à 50 caractères pour éviter le débordement
        formatted_messages = [msg[:50] for msg in messages[-30:]]
        message_text = "\n".join(formatted_messages)
        text_messages.set_text(message_text)
    else:
        text_messages.set_text("Aucun message reçu")
    
    # Affichage du timer
    if st is not None and last_move is not None:
        now = time.time()
        if not robot_stopped and now - last_move > 2:
            robot_stopped = True
            elapsed = last_move - st
            text_timer.set_text(f"Temps final : {elapsed:6.1f} s")
        elif not robot_stopped:
            elapsed = now - st
            text_timer.set_text(f"Temps : {elapsed:6.1f} s")
        else:
            elapsed = last_move - st
            text_timer.set_text(f"Temps final : {elapsed:6.1f} s")
    else:
        text_timer.set_text("Temps : --.- s")

# Animation sans blit pour éviter les plantages
ani = animation.FuncAnimation(fig, update, interval=100, blit=False)
plt.show()
