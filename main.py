# Nano: ssh -X tnp@192.168.1.160

import serial
from pynput import keyboard

arduino = serial.Serial("/dev/ttyACM0", 9600, timeout=1)


def on_press(key):
    try:
        key = key.char
        print("\nYou Entered {0}".format(key))
        if key == "w":
            arduino.write(bytes("w", "utf-8"))
        if key == "t":
            arduino.write(bytes("t", "utf-8"))
        if key == "a":
            arduino.write(bytes("a", "utf-8"))
        if key == "d":
            arduino.write(bytes("d", "utf-8"))
        if key == "z":
            arduino.write(bytes("z", "utf-8"))
        if key == "p":
            arduino.write(bytes("p", "utf-8"))
        if key == "v":
            arduino.write(bytes("v", "utf-8"))
        if key == "b":
            arduino.write(bytes("b", "utf-8"))
        if key == "n":
            arduino.write(bytes("n", "utf-8"))
        if key == "m":
            arduino.write(bytes("m", "utf-8"))
        if key == "i":
            arduino.write(bytes("i", "utf-8"))

    except AttributeError:
        print("\nYou Entered {0}".format(key))
        if key == keyboard.Key.up:
            arduino.write(bytes("up\n", "utf-8"))
        if key == keyboard.Key.esc:
            return False


# print("Start")
# with keyboard.Listener(on_press=on_press) as listener:
#     listener.join()

# print("done")
listener = keyboard.Listener(on_press=on_press)
listener.start()

while True:
    continue
