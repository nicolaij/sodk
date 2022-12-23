# tcp-server.py

import socket
import json
import datetime

HOST = '10.179.40.11'
PORT = 48884
FASTLOADDIR = 'c:\\InSQL\\Data\\DataImport\\'

def parse_msg(msg):
    try:
        return [json.loads(msg)]
    except json.JSONDecodeError as err:
        if err.msg == 'Extra data':
            head = [json.loads(msg[0:err.pos])]
            tail = parse_msg(msg[err.pos:])
            return head + tail
        else:
            return []

# create csv data file
def write_csv(js):
    csvFilename = 'SODK_{}_{:%Y-%m}.csv'.format(js["id"], datetime.datetime.now())
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

# create Wonderware Historian Fast load
def write_historian(js):
        id = js["id"]
        IntouchFilename = '{}{} {:%Y-%m-%d_%H.%M.%S}.csv'.format(FASTLOADDIR, id, datetime.datetime.now())
        try:
            with open(IntouchFilename, 'w', encoding="latin-1") as f:
                f.write("ASCII\n,\n")
                f.write("SODK,1,Server Local,1,1\n")
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
        except Exception as e:
            print('File error! {}'.format(e))
            pass

#начало программы
while True:
    # Создать сокет сервера
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind((HOST, PORT))
        server_socket.listen()

        while True:
            # Обработка запроса подключения
            try:
                conn, addr = server_socket.accept()
                with conn:
                    print('connection from ', addr)
                    while True:
                        message = conn.recv(1024)
                        if not message:
                            break
                        print('recived ({}): {}'.format(len(message), message))
                        for js in parse_msg(message.decode(encoding="latin-1", errors="ignore")):
                            if js:
                                write_csv(js)
                                write_historian(js)

            except Exception as e:
                print('Socket: {}'.format(e))
                continue
