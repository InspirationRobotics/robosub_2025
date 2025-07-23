import serial
import time

port = input("Give file path of your USB port: ")

modem_conn = serial.Serial(port=port,
                           baudrate=9600,
                           parity=serial.PARITY_NONE,
                           stopbits=serial.STOPBITS_ONE,
                           bytesize=serial.EIGHTBITS)
role = None

while role != "sender" and role != "receiver":
    role = input("Sender or receiver?\n").lower()

if role == "sender":
    address = ""
    while len(address) != 3:
        address = input("Input destination address: ")
    int(address)
    for attempt in range(60):
        modem_conn.write(f"$U{address}05Hello".encode())
        time.sleep(5)
elif role == "receiver":
    while True:
        time.sleep(0.1)
        if modem_conn.in_waiting > 0:
            receive_line = modem_conn.readline()
            print(receive_line)
else:
    exit()