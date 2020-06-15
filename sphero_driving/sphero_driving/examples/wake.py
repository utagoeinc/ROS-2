from time import sleep

from pysphero.core import Sphero


def main():
    mac_address = "d6:bc:6a:05:79:b6"
    with Sphero(mac_address=mac_address) as sphero:
        sphero.power.wake()
        sleep(2)
        sphero.power.enter_soft_sleep()


if __name__ == "__main__":
    main()
