import csv
import time
from time import localtime, strftime

import serial
import serial.tools.list_ports

import json

#x = '{ "id":10, "U":500, "R":1501, "rssi":-101}'

DIR = "\\\\database-zrts\\DataImport\\"
#DIR = ""

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
        # print(ser_bytes.hex())
        str = ser_bytes.decode(encoding="latin-1", errors="ignore").strip()
        print("\"" + str + "\"")
        if (str[0] == "{"):
            js = json.loads(str)
            if js["id"] > 1 and js["id"] < 127:
                filename = strftime("%y-%m-%d %H.%M.%S.csv", localtime())
                #f = open(DIR + filename, "w", newline='')
                f = open(DIR + filename, "w")
                f.write("ASCII\n,\n")
                f.write("SODK,1,Server Local,1,1\n")
                cur_time = time.strftime("%Y/%m/%d,%H:%M:%S.000")
                f.write("Sodk_ZRTS_%dU,0,%s,0,%d,192\n" % (js["id"], cur_time, js["U"]))
                f.write("Sodk_ZRTS_%dR,0,%s,0,%d,192\n" % (js["id"], cur_time, js["R"]))
                f.write("Sodk_ZRTS_%drssi,0,%s,0,%d,192\n" % (js["id"], cur_time, js["rssi"]))
                f.write("Sodk_ZRTS_%dU0,0,%s,0,%d,192\n" % (js["id"], cur_time, js["U0"]))
                f.write("Sodk_ZRTS_%dUBatt0,0,%s,0,%d,192\n" % (js["id"], cur_time, js["Ub0"]))
                f.write("Sodk_ZRTS_%dUBatt1,0,%s,0,%d,192\n" % (js["id"], cur_time, js["Ub1"]))
                f.write("Sodk_ZRTS_%dTcpu,0,%s,0,%d,192\n" % (js["id"], cur_time, js["T"]))
                f.close()
        else:
            print(":".join("{:02x}".format(ord(c)) for c in str))


if __name__ == '__main__':
    while True:
        try:
            ser = serial.Serial(port=PORTNAME, baudrate=115200,
                                parity=serial.PARITY_NONE)
            print("Open serial port \"" + ser.name + "\" - OK")
            #time.sleep(1)
            readtofile()
        except Exception as e:
            print("Error:")
            print(e)
            ser.close()

        time.sleep(1)
