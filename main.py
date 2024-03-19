# Nano: ssh -X tnp@192.168.1.160

import serial
from pynput import keyboard

arduino = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)

input_str = ""
def on_press(key):
    global input_str
    try:
        key = key.char
        print("\nYou Entered {0}".format(key))
        if key == "w":
            input_str += key
        if key == "t":
            input_str += key
        if key == "a":
            input_str += key
        if key == "d":
            input_str += key
        if key == "z":
            input_str += key
        if key == "p":
            input_str += key
        if key == "v":
            input_str += key
        if key == "b":
            input_str += key
        if key == "n":
            input_str += key
        if key == "m":
            input_str += key
        if key == "i":
            input_str += key
       
        if key in [".", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9"]:
            input_str += key

    except AttributeError:
        print("\nYou Entered {0}".format(key))
        if key == keyboard.Key.space:
            input_str += " "

        if key == keyboard.Key.enter:
            arduino.write(bytes(input_str, "utf-8"))
            input_str = ""
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
