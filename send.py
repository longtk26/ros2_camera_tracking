import serial
import time

def read_last_line(file_path):
    """Đọc dòng cuối cùng của file"""
    try:
        with open(file_path, "r", encoding="utf-8") as file:
            lines = file.readlines()
            if lines:
                return lines[-1].strip()  # Lấy dòng cuối cùng và loại bỏ khoảng trắng thừa
    except FileNotFoundError:
        print(f"File {file_path} not found. Waiting for file...")
    except Exception as e:
        print(f"Error reading file: {e}")
    return None

def main():
    port = '/dev/ttyUSB0'  # Update to your serial port
    baud_rate = 115200
    file_path = "./data/spec_follow.txt"

    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud_rate,
            timeout=0.01,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )
        print(f"Connected to {port} at {baud_rate} baud.")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return

    try:
        last_sent = None  # Lưu trữ dữ liệu lần gửi trước
        while True:
            data_to_send = read_last_line(file_path)  # Đọc dòng cuối cùng của file
            
            if data_to_send and data_to_send != last_sent:
                ser.write(data_to_send.encode("utf-8"))
                print(f"Sent: {data_to_send}")
                last_sent = data_to_send  # Cập nhật dữ liệu đã gửi


    except KeyboardInterrupt:
        print("Stopping serial data communication.")
    finally:
        ser.close()
        print("Serial port closed.")

if __name__ == "__main__":
    main()
