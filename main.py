# Nano: ssh -X tnp@192.168.1.160

import serial
from pynput import keyboard

arduino = serial.Serial("/dev/ttyACM0", 9600, timeout=1)


def on_press(key):
    print("\nYou Entered {0}".format(key))
    if key == keyboard.Key.up:
        arduino.write(bytes("up\n", "utf-8"))
    if key == keyboard.Key.esc:
        # Stop listener
        return False


# print("Start")
# with keyboard.Listener(on_press=on_press) as listener:
#     listener.join()

# print("done")
listener = keyboard.Listener(on_press=on_press)
listener.start()

while True:
    continue
