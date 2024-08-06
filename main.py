from datetime import datetime
import threading
import serial

serial_port = 'COM3'
baud_rate = 38400

ser = serial.Serial(serial_port, baud_rate, timeout=1)


def date():
    current_datetime = datetime.now()
    formatted_date = current_datetime.strftime("[%Y/%m/%d] ")
    return formatted_date


def save(log):
    file = open("logs.txt", "a")
    file.write(log + "\n")


def send_data():
    while True:
        data_to_send = input()
        data_to_send += "\n"
        ser.write(data_to_send.encode())


def receive_data():
    while True:
        received_data = ser.readline().decode().strip()
        if received_data:
            if received_data.startswith("Digit") or received_data.startswith("Wave") or received_data.startswith("DimStep"):
                print("[INFO]" + date() + received_data)
                save("[INFO]" + date() + received_data)
            elif received_data.startswith("Not"):
                print("[ERR]" + date() + received_data)
                save("[ERR]" + date() + received_data)
            elif received_data.startswith("Critical"):
                print("[WARN]" + date() + received_data)
                save("[WARN]" + date() + received_data)
            else:
                print(received_data)


send_thread = threading.Thread(target=send_data)
receive_thread = threading.Thread(target=receive_data)

send_thread.start()
receive_thread.start()

send_thread.join()
receive_thread.join()

ser.close()