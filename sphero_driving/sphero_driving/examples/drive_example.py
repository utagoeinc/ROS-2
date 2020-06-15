import random
from time import sleep

from pysphero.core import Sphero
from pysphero.driving import Direction

class RosSphero(Sphero):
    def __init__(self, mac_address):
        super(RosSphero, self).__init__(mac_address=mac_address)
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.power.enter_soft_sleep()
        self.ble_adapter.close()

def main():
    mac_address = "d6:bc:6a:05:79:b6"
    with RosSphero(mac_address=mac_address) as sphero:
        sphero.power.wake()

        while True:
            try:
                print('key control: ')
                key = input()
                if key == 'd':
                    speed = random.randint(50, 100)
                    heading = random.randint(0, 360)
                    print(f"Send drive with speed {speed} and heading {heading}")
                    sphero.driving.drive_with_heading(speed, heading, Direction.forward)
                    sleep(1)
                elif key == 's':
                    speed = 0
                    print("stop")
                    sphero.driving.drive_with_heading(speed, heading, Direction.forward)
                    sleep(3)

            except KeyboardInterrupt:
                print('keyboard interrupt')
                break

        # sphero.power.enter_soft_sleep()

if __name__ == "__main__":
    main()
