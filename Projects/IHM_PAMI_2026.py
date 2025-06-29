import serial
import threading
import re
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

# ----- CONFIG -----
BLUETOOTH_PORT = 'COM3'      # adapte ici !
BAUDRATE = 115200
TIMEOUT = 1

# ----- Variables partagées (protégées par thread) -----
terrain_color = "gray"  # par défaut
robot_pos = [0, 0]
target_pos = [0, 0]
lock = threading.Lock()

def parse_line(line):
    global terrain_color, robot_pos, target_pos

    # Terrain color
    m = re.search(r'Terrain (\w+)', line)
    if m:
        with lock:
            c = m.group(1).lower()
            terrain_color = {"jaune": "gold", "bleu": "#2986cc"}.get(c, "gray")
    # Target
    m = re.search(r'TargetX\s*:\s*([\d\.]+).*TargetY\s*:\s*([\d\.]+)', line)
    if m:
        with lock:
            target_pos[0] = float(m.group(1))
            target_pos[1] = float(m.group(2))
    # Robot
    m = re.search(r'RobotX\s*:\s*([\d\.]+).*RobotY\s*:\s*([\d\.]+)', line)
    if m:
        with lock:
            robot_pos[0] = float(m.group(1))
            robot_pos[1] = float(m.group(2))

def serial_thread():
    ser = serial.Serial(BLUETOOTH_PORT, BAUDRATE, timeout=TIMEOUT)
    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                # print("DEBUG:", line)  # décommente si tu veux voir les messages bruts
                parse_line(line)
    except Exception as e:
        print("Erreur série :", e)
    finally:
        ser.close()

# ----- Interface graphique -----
fig, ax = plt.subplots(figsize=(10, 7))
plt.title("Visualisation PAMI - Position robot et cible")
terrain_patch = patches.Rectangle((0,0), 300, 200, linewidth=2, edgecolor='black', facecolor="gray")
ax.add_patch(terrain_patch)
robot_dot, = ax.plot([], [], 'o', color='red', markersize=16, label="Robot")
target_dot, = ax.plot([], [], '*', color='lime', markersize=20, label="Cible")
text_robot = ax.text(5, 195, "", fontsize=13, color="red", weight="bold")
text_target = ax.text(5, 180, "", fontsize=13, color="green", weight="bold")
text_color = ax.text(5, 165, "", fontsize=13, color="black", weight="bold")

ax.set_xlim(0, 300)
ax.set_ylim(0, 200)
ax.set_xlabel("X (cm)")
ax.set_ylabel("Y (cm)")
plt.legend(loc="lower right")

def update(_):
    # Affichage dynamique des données
    with lock:
        robotx, roboty = robot_pos
        targetx, targety = target_pos
        col = terrain_color
    terrain_patch.set_facecolor(col)
    robot_dot.set_data([robotx], [roboty])
    target_dot.set_data([targetx], [targety])
    text_robot.set_text(f"Robot : ({robotx:.1f}, {roboty:.1f})")
    text_target.set_text(f"Cible : ({targetx:.1f}, {targety:.1f})")
    txt = f"Terrain : {'Jaune' if col=='gold' else ('Bleu' if col=='#2986cc' else 'Inconnu')}"
    text_color.set_text(txt)
    return robot_dot, target_dot, text_robot, text_target, terrain_patch, text_color

# ----- Démarrage du thread série -----
threading.Thread(target=serial_thread, daemon=True).start()

# Animation matplotlib (20 fps)
ani = animation.FuncAnimation(fig, update, interval=50, blit=True)
plt.tight_layout()
plt.show()
