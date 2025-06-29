import serial
import threading
import re
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys
import time

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

def parse_line(line):
    global terrain_str, robot_pos, target_pos, start_time, last_robot_move_time, robot_stopped

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
            # Détecte si le robot a bougé
            if not robot_history or (new_x, new_y) != robot_history[-1]:
                last_robot_move_time = time.time()
                robot_stopped = False
            robot_pos[0] = new_x
            robot_pos[1] = new_y
            robot_history.append((new_x, new_y))


def serial_thread():
    try:
        ser = serial.Serial(BLUETOOTH_PORT, BAUDRATE, timeout=TIMEOUT)
        connection_ok.set()
    except Exception as e:
        print(f"ERREUR : Impossible d’ouvrir le port {BLUETOOTH_PORT} : {e}")
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
        ser.close()

# ----- Thread série -----
t = threading.Thread(target=serial_thread, daemon=True)
t.start()
# Attend la connexion :
connection_ok.wait(timeout=3)

if not connection_ok.is_set():
    print(f"\nConnexion impossible sur {BLUETOOTH_PORT}. Aucune interface graphique n’est lancée.\n")
    sys.exit(1)

# ----- Interface graphique -----
fig, ax = plt.subplots(figsize=(10, 7))
plt.title("PAMI : Position robot et cible")
ax.set_xlim(0, 200)
ax.set_ylim(0, 300)
ax.set_xlabel("Y (cm)")
ax.set_ylabel("X (cm)")

robot_dot, = ax.plot([], [], 'o', color='red', markersize=16, label="Robot (actuel)")
target_dot, = ax.plot([], [], '*', color='lime', markersize=20, label="Cible (actuelle)")
robot_hist_plot, = ax.plot([], [], '.', color='crimson', alpha=0.25, markersize=8, label="Trajet robot")
target_hist_plot, = ax.plot([], [], '.', color='deepskyblue', alpha=0.7, markersize=18, label="Historique cibles")

text_robot = ax.text(5, 285, "", fontsize=13, color="red", weight="bold")
text_target = ax.text(5, 272, "", fontsize=13, color="green", weight="bold")
text_terrain = ax.text(5, 259, "", fontsize=13, color="black", weight="bold")
text_timer = ax.text(120, 285, "", fontsize=13, color="blue", weight="bold")  # Timer plus bas

plt.legend(loc="lower right")
plt.subplots_adjust(top=0.93)  # Laisse de la marge en haut

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
    # Axes inversés :
    robot_dot.set_data([ry], [rx])
    target_dot.set_data([ty], [tx])
    if rob_hist:
        ys, xs = zip(*rob_hist)
        robot_hist_plot.set_data(xs, ys)
    if tar_hist:
        ys, xs = zip(*tar_hist)
        target_hist_plot.set_data(xs, ys)
    text_robot.set_text(f"Robot : ({rx:.1f}, {ry:.1f})")
    text_target.set_text(f"Cible : ({tx:.1f}, {ty:.1f})")
    text_terrain.set_text(f"Terrain : {terr}")
    # Affichage du timer
    if st is not None and last_move is not None:
        now = time.time()
        # Si le robot n'a pas bougé depuis 2 secondes, on arrête le timer
        if not robot_stopped and now - last_move > 2:
            robot_stopped = True
            elapsed = last_move - st
            text_timer.set_text(f"Temps final : {elapsed:6.1f} s")
        elif not robot_stopped:
            elapsed = now - st
            text_timer.set_text(f"Temps : {elapsed:6.1f} s")
        else:
            elapsed = last_move - st
            text_timer.set_text(f"Temps final : {elapsed:6.1f} s")
    else:
        text_timer.set_text("Temps : --.- s")
    return robot_dot, target_dot, robot_hist_plot, target_hist_plot, text_robot, text_target, text_terrain, text_timer

ani = animation.FuncAnimation(fig, update, interval=50, blit=True)
plt.tight_layout()
plt.show()
