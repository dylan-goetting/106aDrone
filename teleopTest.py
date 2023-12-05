from pynput.keyboard import Key, Listener

def show(key):

    if key == Key.tab:
        print("good")

    if key != Key.tab:
        print("try again")

    # by pressing 'delete' button 
    # you can terminate the loop 
    if key == Key.delete:
        return False

# Collect all event until released
with Listener(on_press = show) as listener:
    listener.join()