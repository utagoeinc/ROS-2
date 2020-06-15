from time import sleep
from typing import Dict

from pysphero.core import Sphero
from pysphero.device_api.sensor import CoreTime, Quaternion

x = 0.0

def notify_callback(data: Dict):
    # info = ", ".join("{:1.2f}".format(data.get(param)) for param in Quaternion)
    # print(f"[{data.get(CoreTime.core_time):1.2f}] Quaternion (x, y, z, w): {info}")
    # print("=" * 60)
    global x
    x = data.get(Quaternion.x)



def main():
    global x
    mac_address = "d6:bc:6a:05:79:b6"
    with Sphero(mac_address=mac_address) as sphero:
        sphero.power.wake()
        sphero.sensor.set_notify(notify_callback, CoreTime, Quaternion)
        for i in range(10):
            print("x: {0}".format(x))
            sleep(1)
        # sleep(5)
        sphero.sensor.cancel_notify_sensors()
        sphero.power.enter_soft_sleep()


if __name__ == "__main__":
    main()
