import serial
import threading
import re
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys

# ----- CONFIG -----
BLUETOOTH_PORT = 'COM3'      # adapte ici !
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

def parse_line(line):
    global terrain_str, robot_pos, target_pos

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
            robot_pos[0] = float(m.group(1))
            robot_pos[1] = float(m.group(2))
            robot_history.append(tuple(robot_pos))

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
ax.set_xlim(0, 200)   # X sur vertical, Y sur horizontal
ax.set_ylim(0, 300)
ax.set_xlabel("Y (cm)")
ax.set_ylabel("X (cm)")

robot_dot, = ax.plot([], [], 'o', color='red', markersize=16, label="Robot (actuel)")
target_dot, = ax.plot([], [], '*', color='lime', markersize=20, label="Cible (actuelle)")
robot_hist_plot, = ax.plot([], [], '.', color='crimson', alpha=0.25, markersize=8, label="Trajet robot")
target_hist_plot, = ax.plot([], [], '.', color='deepskyblue', alpha=0.15, markersize=10, label="Historique cibles")

text_robot = ax.text(5, 295, "", fontsize=13, color="red", weight="bold")
text_target = ax.text(5, 282, "", fontsize=13, color="green", weight="bold")
text_terrain = ax.text(5, 269, "", fontsize=13, color="black", weight="bold")

plt.legend(loc="lower right")

def update(_):
    with lock:
        rx, ry = robot_pos
        tx, ty = target_pos
        rob_hist = list(robot_history)
        tar_hist = list(target_history)
        terr = terrain_str
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
    return robot_dot, target_dot, robot_hist_plot, target_hist_plot, text_robot, text_target, text_terrain

ani = animation.FuncAnimation(fig, update, interval=50, blit=True)
plt.tight_layout()
plt.show()
