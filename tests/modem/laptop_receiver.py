import serial
import time

# --- Change these to match your laptop's modem configuration ---
SERIAL_PORT = "COM5"   # Update to your actual device (check with ls /dev/ttyUSB*)
BAUDRATE = 9600

def main():
    print(f"[INFO] Opening modem on {SERIAL_PORT} ...")
    with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
        print("[INFO] Listening for incoming modem messages. Press Ctrl+C to exit.\n")
        while True:
            if ser.in_waiting > 0:
                msg = ser.readline().decode(errors='ignore').strip()
                if msg:
                    print(f"[RECV] {msg}")
                    
                    # Optional: send acknowledgment or reply
                    # if msg == "ROLL":
                    #     ser.write(b"ROLL_DONE\n")
                    #     print("[SEND] ROLL_DONE")
            time.sleep(0.1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Exiting receiver script.")
