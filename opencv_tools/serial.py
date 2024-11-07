import serial


class Serial:
    def __init__(self) -> None:
        pass

    def connect(self, port: str, baudrate: int) -> None:
        # Connect to the serial port
        self.serial_port = serial.Serial(port, baudrate)
        self.serial_port.timeout = 1  # Set timeout for reading data
    
    def read_line(self) -> str:
        # Read a line from the serial port
        return self.serial_port.readline().decode('utf-8').strip()
    
    def send_command(self, command: str) -> None:
        # Send a command to the serial port
        self.serial_port.write(command.encode('utf-8'))