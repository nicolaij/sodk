import csv
import time
from time import localtime, strftime

import serial
import serial.tools.list_ports

import json

x = '{ "id":10, "U":500, "R":1501, "rssi":-101}'

PORTNAME = "COM7"

ports = serial.tools.list_ports.comports()

for port, desc, hwid in (ports):
    print("{}: {} [{}]".format(port, desc, hwid))

# import serial
# ports = serial.tools.list_ports.comports(include_links=True)
# for port in ports :
#    print(port.device)


def readtofile():
    while True:
        ser_bytes = ser.readline()
        print(ser_bytes.hex())
        str = ser_bytes.decode(encoding="latin-1", errors="ignore").strip()
        print("\"" + str + "\"")
        js = json.loads(x)
        if js["id"] > 1 and js["id"] < 127:
            filename = strftime("%y-%m-%d %H.%M.%S.csv", localtime())
            f = open(filename, "w", newline='')
            f.write("ASCII\n,\n")
            f.write("SODK,1,Server Local,1,1\n")
            cur_time = time.strftime("%Y/%m/%d,%H:%M:%S.000")
            f.write("Sodk_ZRTS_%dU,0,%s,1,%d,192\n" % (js["id"], cur_time, js["U"]))
            f.write("Sodk_ZRTS_%dR,0,%s,1,%d,192\n" % (js["id"], cur_time, js["R"]))
            f.write("Sodk_ZRTS_%drssi,0,%s,1,%d,192\n" % (js["id"], cur_time, js["rssi"]))
            f.close()
            #writer = csv.writer(f, delimiter=",")
            #writer.writerow(["Sodk_ZRTS_" + js["id"] + "R","0",time.strftime("%Y/%m/%d,%H:%M:%S.000"), str, js["U"], js["R"]])
            #writer.writerow([time.strftime("%Y/%m/%d,%H:%M:%S.000"), str, js["id"], js["U"], js["R"]])

if __name__ == '__main__':
    while True:
        try:
            ser = serial.Serial(port=PORTNAME, baudrate=115200,
                                parity=serial.PARITY_NONE)
            print("Open serial port \"" + ser.name + "\" - OK")
            readtofile()
        except Exception as e:
            print(e)
            print("Serial port - BUSY")
            ser.close()

        time.sleep(1)
