import serial
import time

def main():
    # Configure the serial port (update the port name as needed)
    port = '/dev/ttyUSB0'  # Update to your serial port, e.g., COM3 for Windows
    baud_rate = 19200  # Match this with your device's baud rate

    # Open the serial port
    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"Connected to {port} at {baud_rate} baud.")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return

    try:
        while True:
            # Read data from the serial port
            try:
                if ser.isOpen():
                    data = ser.readline().decode('utf-8').strip()
                    print(f"Received data: {data}")
            except UnicodeDecodeError as e:
                print(f"Error decoding data: {e}")

    except KeyboardInterrupt:
        print("Stopping serial data read.")
    finally:
        ser.close()
        print("Serial port closed.")

if __name__ == "__main__":
    main()
