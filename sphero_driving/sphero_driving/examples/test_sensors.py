from time import sleep
from typing import Dict

from pysphero.core import Sphero
from pysphero.device_api.sensor import CoreTime, Quaternion


flag = True
x = 0.0

import signal
def signalHandlerCallback(sig_no, frame):
    global flag
    print('Signal catch.')
    flag = False
signal.signal(signal.SIGINT, signalHandlerCallback)



def notify_callback(data: Dict):
    # info = ", ".join("{:1.2f}".format(data.get(param)) for param in Quaternion)
    # print(f"[{data.get(CoreTime.core_time):1.2f}] Quaternion (x, y, z, w): {info}")
    # print("=" * 60)
    global x
    x = data.get(Quaternion.x)



def main():
    global x, flag
    mac_address = "d6:bc:6a:05:79:b6"
    with Sphero(mac_address=mac_address) as sphero:
        sphero.power.wake()
        sphero.sensor.set_notify(notify_callback, CoreTime, Quaternion)

    print('start')
    # for i in range(10):
    while True:
        if(flag):
            print("x: {0}".format(x))
            sleep(1)
        # sleep(5)
    with Sphero(mac_address=mac_address) as sphero:
        sphero.sensor.cancel_notify_sensors()
        sphero.power.enter_soft_sleep()


if __name__ == "__main__":
    main()
