from pynput.keyboard import Key, Listener
import keyboard

while True:

    print(keyboard.read_key())
    if keyboard.read_key() == "a":
        break