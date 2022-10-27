# tcp-server.py

from socket import *
import json
import datetime

host = '10.179.40.20'
port = 48884
bufferSize = 1024

id = ''
IntouchFastLoadDir = 'c:\\InSQL\\Data\\DataImport\\'

class CustomError(Exception):
    pass

while True:
    # Создать сокет сервера
    server_socket = socket(AF_INET, SOCK_DGRAM)
    #   s et et
    server_addr = (host, port)
    server_socket.bind(server_addr)

    # Обработка запроса подключения
    while True:
        bytesAddressPair = server_socket.recvfrom(bufferSize)

        message = bytesAddressPair[0]
        address = bytesAddressPair[1]

        clientMsg = "{}:{}".format(address, message)
        print(clientMsg)
        
        try:
            js = json.loads(message.decode(encoding="latin-1", errors="ignore"))
            id = js["id"]
        except Exception as e:
            print('Data format error! {}'.format(e))
            continue

        #create csv data file
        csvFilename = 'SODK_{}_{:%Y-%m}.csv'.format(id, datetime.datetime.now())
        try:
            with open(csvFilename, 'r', newline='') as csvfile:
                n = csvfile.read(1)
        except:
            with open(csvFilename, 'w', newline='') as csvfile:
                csvfile.write('Datetime,Counter,U,R,U0,Ub0,Ub1,T,RSSI,in\n')
            pass
        
        try:
            with open(csvFilename, 'a', newline='') as csvfile:
                csvfile.write('{},{},{},{},{},{},{},{},{},{}\n'.format(js["dt"], js["num"], js["U"], js["R"], js["U0"], js["Ub0"], js["Ub1"], js["T"], js["rssi"], js["in"]))
        except Exception as e:
            print('CSV file error! {}'.format(e))
            pass

        #create Intouch Fast load 
        IntouchFilename = '{}{} {:%Y-%m-%d_%H.%M.%S}.csv'.format(IntouchFastLoadDir, id, datetime.datetime.now())
        with open(IntouchFilename, 'w', encoding="latin-1") as f:
            f.write("ASCII\n,\n")
            f.write("SODK,1,Server Local,1,1\n")
            #time_and_date = '{:%Y/%m/%d,%H:%M:%S}'.format(datetime.datetime.now())
            dt = datetime.datetime.strptime(js["dt"], "%Y-%m-%d %H:%M:%S")
            time_and_date = '{:%Y/%m/%d,%H:%M:%S}.000'.format(dt)
            f.write("Sodk_ZRTS{}_U,0,{},0,{},192\n".format(id, time_and_date, js["U"]))
            f.write("Sodk_ZRTS{}_R,0,{},0,{},192\n".format(id, time_and_date, js["R"]))
            f.write("Sodk_ZRTS{}_U0,0,{},0,{},192\n".format(id, time_and_date, js["U0"]))
            f.write("Sodk_ZRTS{}_UBatt0,0,{},0,{},192\n".format(id, time_and_date, js["Ub0"]))
            f.write("Sodk_ZRTS{}_UBatt1,0,{},0,{},192\n".format(id, time_and_date, js["Ub1"]))
            f.write("Sodk_ZRTS{}_Tcpu,0,{},0,{},192\n".format(id, time_and_date, js["T"]))
            f.write("Sodk_ZRTS{}_rssi,0,{},0,{},192\n".format(id, time_and_date, js["rssi"]))
            f.write("Sodk_ZRTS{}_in,0,{},0,{},192\n".format(id, time_and_date, js["in"]))
        
        #IntouchFilename = '{} {:%Y-%m-%d_%H.%M.%S}.csv'.format(id, datetime.datetime.now())
        #with open(IntouchFilename, 'w', encoding="latin-1") as f:
        #    f.write("ASCII\n,\n")
        #    f.write("SODK,1,Server Local,1,1\n")
        #    #time_and_date = '{:%Y/%m/%d,%H:%M:%S}'.format(datetime.datetime.now())
        #    dt = datetime.datetime.strptime(js["dt"], "%Y-%m-%d %H:%M:%S")
        #    time_and_date = '{:%Y/%m/%d,%H:%M:%S}.000'.format(dt)
        #    f.write("Sodk_ZRTS{}_U,0,{},0,{},192\n".format(id, time_and_date, js["U"]))
        #    f.write("Sodk_ZRTS{}_R,0,{},0,{},192\n".format(id, time_and_date, js["R"]))
        #    f.write("Sodk_ZRTS{}_U0,0,{},0,{},192\n".format(id, time_and_date, js["U0"]))
        #    f.write("Sodk_ZRTS{}_UBatt0,0,{},0,{},192\n".format(id, time_and_date, js["Ub0"]))
        #    f.write("Sodk_ZRTS{}_UBatt1,0,{},0,{},192\n".format(id, time_and_date, js["Ub1"]))
        #    f.write("Sodk_ZRTS{}_Tcpu,0,{},0,{},192\n".format(id, time_and_date, js["T"]))
        #    f.write("Sodk_ZRTS{}_rssi,0,{},0,{},192\n".format(id, time_and_date, js["rssi"]))
