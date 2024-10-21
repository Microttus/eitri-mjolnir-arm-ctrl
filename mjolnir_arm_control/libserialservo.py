import serial
import time


class ServoController:
    def __init__(self, port: str, baud_rate: int = 9600, timeout: float = 1.0):
        """
        Initializes a serial connection to the TinyPICO.

        Args:
            port: The port to which the TinyPICO is connected (e.g., 'COM3' or '/dev/ttyUSB0').
            baud_rate: The baud rate for serial communication.
            timeout: Timeout for the serial connection.
        """
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial_connection = None

    def connect(self):
        """Establishes a serial connection to the TinyPICO."""
        try:
            self.serial_connection = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
            time.sleep(2)
            print(f"Connected to ServoDevice on {self.port} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            print(f"Error connecting to TinyPICO: {e}")
            raise

    def send_servo_values(self, servo_values: list):
        """
        Sends servo motor values to the TinyPICO.

        Arge:
            servo_values: A liust of six integers representing the servo motor positions
        """
        if len(servo_values) != 6:
            raise ValueError("You must provide exactly 6 servo values.")

        if self.serial_connection is None or not self.serial_connection.is_open:
                raise ConnectionError("No open serial connection to TinyPICO.")

        # Convert the list of servo values into a comma-separated string
        servo_data = ','.join(map(str, servo_values)) + '\n'

        try:
            self.serial_connection.write(servo_data.encode())
            time.sleep(0.2)
            print(f"Sent servo data: {servo_data}")
        except serial.SerialTimeoutException as e:
            print(f"Failed to send data: {e}")

    def close(self):
        """Closes th serial connection."""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print("Serial connection closed.")