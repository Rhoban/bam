import zmq
import copy
import numpy as np
import threading
import os
import time
import etherban_pb2 as messages
from trajectory import cubic_interpolate


class Client:
    def __init__(self, host: str):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(f"tcp://{host}:7332")
        self.running = True
        self.lock = threading.Lock()
        self.wait_lock = threading.Condition()

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

    def position_control(
        self,
        index: int,
        target_position: float,
        target_velocity: float = 0.0,
        kp: float = 10.0,
        max_amps: float = 12.0,
    ):
        status = self.statuses[index]
        position_error = target_position - status["position"]
        velocity_error = target_velocity - status["velocity"]
        amps = position_error * kp + 2 * np.sqrt(kp) * velocity_error
        amps = max(-max_amps, min(max_amps, amps))
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

            self.wait_lock.acquire()
            self.wait_lock.notify_all()
            self.wait_lock.release()

    def sync(self):
        self.wait_lock.acquire()
        self.wait_lock.wait()
        self.wait_lock.release()

    def wait_stability(self, index: int):
        positions = []
        while True:
            status = self.statuses[index]
            positions.append(status["position"])
            positions = positions[-100:]
            if len(positions) == 100 and np.std(positions) < 1e-5:
                break
            self.sync()

    def goto_safe(self, index: int, target: float, duration: float = 3.0):
        status = self.statuses[index]
        start_pos = status["position"]
        t0 = time.time()
        t = 0

        while t < duration:
            t = time.time() - t0
            current_target = cubic_interpolate(
                [[0.0, start_pos, 0], [duration, target, 0]], t
            )
            self.set_order(0, "position", current_target)
            self.sync()

    def run_background(self):
        thread = threading.Thread(target=self.run)
        thread.start()

        self.sync()

    def stop(self):
        self.running = False


if __name__ == "__main__":
    client = Client("localhost")
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
