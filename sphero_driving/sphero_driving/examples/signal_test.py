import signal
from time import sleep

flag = True
def signalHandlerCallback(sig_no, frame):
    global flag
    print('Signal catch.')
    flag = False
signal.signal(signal.SIGINT, signalHandlerCallback)

if __name__ == "__main__":
    while flag:
      print('test')
      sleep(1)
