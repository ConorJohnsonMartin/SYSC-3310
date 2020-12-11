import time
import serial
import tkinter as tk
import threading

window = tk.Tk()
serial = serial.Serial("COM5", 9600)

def increment():
    serial.write(b"1")


def decrement():
    serial.write(b"2")


def read_state():
    while True:
        print(int.from_bytes(serial.read(1), byteorder="big"))


increment_button = tk.Button(window, text="increment", command=increment, font=("Helvetica", 20))
decrement_button = tk.Button(window, text="decrement", command=decrement, font=("Helvetica", 20))
stop_button = tk.Button(window, text="stop", command=exit, font=("Helvetica", 20))
increment_button.grid(row=1, column=1, padx=10, pady=20)
decrement_button.grid(row=2, column=2, padx=10, pady=20)
stop_button.grid(row=3, column=3, padx=10, pady=20)
window.configure(background="cyan")
x = threading.Thread(target=read_state, args=(), daemon=True)
x.start()
while True:
    window.mainloop()