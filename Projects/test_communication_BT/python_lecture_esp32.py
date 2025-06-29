import serial
import time

BLUETOOTH_PORT = 'COM7'
BAUDRATE = 115200

while True:
    try:
        ser = serial.Serial(BLUETOOTH_PORT, BAUDRATE, timeout=2)
        print("Port ouvert.")
        last_line_time = time.time()
        last_reset = time.time()
        while True:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print("Reçu:", line)
                    last_line_time = time.time()
                # RESET toutes les 10s
                now = time.time()
                if now - last_reset > 10:
                    ser.write(b'RESET\n')
                    print("RESET envoyé")
                    last_reset = now
                # Si plus rien depuis 6s → break pour relancer la boucle
                if now - last_line_time > 6:
                    print("Plus de data, tentative de reconnexion...")
                    break
            except Exception as e:
                print("Erreur de lecture/écriture:", e)
                break
        ser.close()
        print("Port fermé. Réessai dans 2s...")
        time.sleep(2)
    except Exception as e:
        print("Impossible d'ouvrir le port:", e)
        time.sleep(2)
