import serial
import time

class LinearActuator:
    def __init__(self, port: str= "COM3", baudrate: int = 921600):
        self.port = port
        self.baudrate = baudrate

        self.ser = serial.Serial(port, baudrate)
        if not self.ser.is_open:
            raise Exception("Failed to open serial port")
    
    def envoi_commande(self, commande: str):
        self.ser.write((commande + "\n").encode('utf-8'))
        # time.sleep(0.01)
    
    def read_response(self, timeout: float = 0.020) -> list[bytes]:
        response = b''
        over = False
        start = time.monotonic()

        while not over and time.monotonic() - start < timeout:
            while self.ser.in_waiting:
                byte = self.ser.read()
                if byte == b'$':
                    over = True
                else:
                    response += byte
            else:
                time.sleep(1e-3)

        return response.strip().split(b"\n")

    def set_p_gain(self, gain: int):
        self.envoi_commande(f"kp={gain}")
        self.ser.reset_output_buffer()
       
    def set_torque(self, enable: bool):
        if not enable:
            self.envoi_commande("relay 1")
            self.ser.reset_output_buffer()
        else:
            self.envoi_commande("relay 0")
            self.ser.reset_output_buffer()


    def set_goal_position(self, position: float):
        self.envoi_commande(f"servo {position}")
        self.ser.reset_output_buffer()

    def read_value(self, response: list[bytes], key: bytes) -> float|None:
        for line in response:
            if b':' in line:
                line_key, value = line.strip().split(b': ', 2)
                if line_key == key:
                    try:
                        return float(value)
                    except ValueError:
                        return None
                
        return None
        
    def read_data(self):
        self.envoi_commande("l") 
        position= self.read_value(self.read_response(), b"l")
        self.envoi_commande("mag") 
        encoder_value= self.read_value(self.read_response(), b"Value")

        return {
            "position": position,
            "encoder": encoder_value,
        }
    