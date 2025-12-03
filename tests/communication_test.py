
import serial
import time

PORT = "COM7"   # change for your system
BAUD = 9600

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)
print("Connected to Arduino.")

def set_led(led_number, state):
    """
    led_number: 1-5
    state: "ON" or "OFF"
    """
    cmd = f"LED {led_number} {state}\n"
    ser.write(cmd.encode())

if __name__ == "__main__":
    while True:
        cmd = input("Example: '1 on', '3 off', 'quit': ").strip().lower()

        if cmd == "quit":
            break

        try:
            led_num, action = cmd.split()
            led_num = int(led_num)

            if action not in ("on", "off"):
                print("Action must be on/off")
                continue

            if led_num < 1 or led_num > 5:
                print("LED must be 1–5")
                continue

            set_led(led_num, action.upper())

        except:
            print("Invalid format.")
