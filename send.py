import serial
import time

def main():
    port = '/dev/ttyUSB0'  # Update to your serial port
    baud_rate = 115200

    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud_rate,
            timeout=2,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )
        print(f"Connected to {port} at {baud_rate} baud.")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return

    try:
        count = 0
        while True:
            if ser.isOpen():
                # Send data
                data_to_send = f"Hello, World! {count}\n"
                ser.write(data_to_send.encode("utf-8"))
                print(f"Sent: {data_to_send.strip()}")

                # Read response
                response = ser.readline()
                print(f"Received: {response}")

                count += 1
                time.sleep(1)
            else:
                print("Serial port is not open.")
    except KeyboardInterrupt:
        print("Stopping serial data communication.")
    finally:
        ser.close()
        print("Serial port closed.")

if __name__ == "__main__":
    main()
