import serial

BLUETOOTH_PORT = 'COM3'   # adapte selon ton PC !
BAUDRATE = 115200         # identique au SerialBT côté ESP32

ser = serial.Serial(BLUETOOTH_PORT, BAUDRATE, timeout=1)
print("Attente des données Bluetooth...")

try:
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print("Reçu:", line)
except KeyboardInterrupt:
    print("Arrêt.")
finally:
    ser.close()
