import time
from time import gmtime, strftime
import serial
import serial.tools.list_ports
import csv

PORTNAME = "COM7"

ports = serial.tools.list_ports.comports()

for port, desc, hwid in (ports):
    print("{}: {} [{}]".format(port, desc, hwid))

# import serial
# ports = serial.tools.list_ports.comports(include_links=True)
# for port in ports :
#    print(port.device)


def portIsUsable(portName):
    try:
        ser = serial.Serial(port=portName)
        return True
    except:
        return False


def readtofile():
    filename = strftime("%y-%m-%d %H.%M.%S.csv", gmtime())
    while True:
        ser_bytes = ser.readline()
        str = ser_bytes.decode("latin-1", "ignore").strip()
        print(str)
        with open(filename, "a") as f:
            writer = csv.writer(f, delimiter=",")
            writer.writerow([time.time(), str])


if __name__ == '__main__':
    while True:
        try:
            ser = serial.Serial(port=PORTNAME)
            print("Serial port - OK")
            readtofile()
        except Exception as e:
            print(e)
            ser.close()
            print("Serial port - BUSY")

        time.sleep(1)
