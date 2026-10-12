import serial
import threading
import argparse
from prompt_toolkit import PromptSession
from prompt_toolkit.patch_stdout import patch_stdout

def read_serial(ser):
    buffer = b""
    while True:
        
        if ser.in_waiting:
            raw = ser.read(ser.in_waiting)
            buffer += raw

            
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                try:
                    text = line.decode('utf-8', errors='ignore')
                except:
                    text = line.decode('latin1', errors='ignore')
                print(f"\rReceived: {text}")
                
def main():
    parser = argparse.ArgumentParser(description='Serial JSON Communication with prompt_toolkit')
    parser.add_argument('port', type=str, help='Serial port name (e.g., COM3 or /dev/ttyUSB0)')
    args = parser.parse_args()

    ser = serial.Serial(args.port, baudrate=115200, timeout=0.1)

    t = threading.Thread(target=read_serial, args=(ser,))
    t.daemon = True
    t.start()

    session = PromptSession()

    print("Connected. You can enter commands (Ctrl+C to exit).")

    try:
        with patch_stdout():
            while True:
                cmd = session.prompt('>>> ')
                if not cmd.strip():
                    continue
                ser.write((cmd + '\n').encode())
                print(f"Sent: {cmd}")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()