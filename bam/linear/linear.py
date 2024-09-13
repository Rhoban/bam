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
        self.read_response()
       
    def set_torque(self, enable: bool):
        if not enable:
            self.envoi_commande("relay 1")
        else:
            self.envoi_commande("relay 0")
        self.read_response()


    def set_goal_position(self, position: float):
        self.envoi_commande(f"servo {position}")
        self.read_response()

    def read_value(self, response: list[bytes], key: bytes) -> float|None:
        for line in response:
            parts = line.strip().split(b': ', 1)
            if len(parts) == 2:
                line_key, value = parts
                if line_key == key:
                    try:
                        return float(value)
                    except ValueError:
                        return None
                
        return None
        
    def read_data(self):
        self.envoi_commande("l") 
        response = self.read_response()
        position= self.read_value(response, b"l")
        encoder_value= self.read_value(response, b"alpha")

        return {
            "position": position,
            "encoder": encoder_value,
        }
    