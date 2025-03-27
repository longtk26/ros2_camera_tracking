import serial
import time

def main():
    # Configure the serial port (update the port name as needed)
    port = '/dev/ttyUSB0'  # Update to your serial port
    baud_rate = 115200  # Match this with your device's baud rate
    file_path = "./data/imu_data.txt"  # File to store IMU data

    # Open the serial port
    try:
        ser = serial.Serial(port, baud_rate, timeout=0.01)
        print(f"Connected to {port} at {baud_rate} baud.")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return

    try:
        with open(file_path, "a") as file:  # Open file in append mode
            while True:
                # Read data from the serial port
                try:
                    if ser.isOpen():
                        data = ser.readline().decode('utf-8').strip()
                        if data:
                            print(f"Received data: {data}")
                            file.write(data + "\n")  # Write to file
                            file.flush()  # Ensure data is written immediately
                except UnicodeDecodeError as e:
                    print(f"Error decoding data: {e}")

    except KeyboardInterrupt:
        print("\nStopping serial data read.")
    finally:
        ser.close()
        print("Serial port closed.")

if __name__ == "__main__":
    main()
