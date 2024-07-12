import time
from linear import LinearActuator

l = LinearActuator()

def bench():
    t = time.monotonic()
    for k in range(100):
        l.read_data()  
    print((time.monotonic() - t)/100.)
bench()