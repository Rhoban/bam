import zmq
import copy
import numpy as np
import threading
import os
import time
import etherban_pb2 as messages


class Client:
    def __init__(self, host: str):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(f"tcp://{host}")
        self.running = True
        self.lock = threading.Lock()

        self.orders = {}
        self.statuses = {}

    def set_order(self, index: int, mode="position", target=0.0):
        self.lock.acquire()
        if index not in self.orders:
            self.orders[index] = messages.Order()

        order = self.orders[index]
        order.device = index

        if mode == "position":
            order.operation_mode = messages.POSITION
        elif mode == "velocity":
            order.operation_mode = messages.VELOCITY
        elif mode == "torque":
            order.operation_mode = messages.TORQUE
        order.target = target
        self.lock.release()

    def position_control(self, index: int, target_position: float, target_velocity: float = 0.0, kp: float = 10.0):
        status = self.statuses[index]
        position_error = target_position - status["position"]
        velocity_error = target_velocity - status["velocity"]
        amps = position_error * kp + 2 * np.sqrt(kp) * velocity_error
        amps = max(-10, min(10, amps))
        self.set_order(index, "torque", amps)

    def stop(self, index: int):
        self.set_order(index, "torque", 0.0)

    def send(self):
        self.lock.acquire()
        message = messages.Request()

        # Adding orders to the packet
        for index in self.orders:
            order = self.orders[index]
            message.orders.append(order)

        # Send the message
        self.socket.send(message.SerializeToString())

        # Receive the response
        response = self.socket.recv()
        response_message = messages.Response()
        response_message.ParseFromString(response)

        for status in response_message.statuses:
            self.statuses[status.device] = {
                "target_position": status.target_position,
                "torque_demand": status.torque_demand,
                "velocity_demand": status.velocity_demand,
                "position": status.present_position,
                "velocity": status.present_velocity,
                "current": status.present_current,
            }

        self.lock.release()

        if response_message.has_fault:
            print("Server has fault, clearing it and aborting")
            exit(0)

    def get_statuses(self):
        self.lock.acquire()
        statuses = copy.deepcopy(self.statuses)
        self.lock.release()

        return statuses

    def run(self):
        t0 = time.time()
        while self.running:
            while time.time() - t0 < 0.001:
                time.sleep(1e-4)
            t0 += 0.001

            self.send()

    def run_background(self):
        thread = threading.Thread(target=self.run)
        thread.start()

    def stop(self):
        self.running = False


if __name__ == "__main__":
    client = Client("localhost:7332")
    start = time.time()
    client.run_background()

    while True:
        os.system("clear")
        print(f"{len(client.statuses)} devices")
        for k in client.statuses:
            status = client.statuses[k]
            print(f"Device {k}")
            print(f" - Target Position: {status['target_position']}")
            print(f" - Torque Demand: {status['torque_demand']}")
            print(f" - Velocity Demand: {status['velocity_demand']}")
            print(f" - Position: {status['position']}")
            print(f" - Velocity: {status['velocity']}")
            print(f" - Current: {status['current']}")
        time.sleep(0.1)
