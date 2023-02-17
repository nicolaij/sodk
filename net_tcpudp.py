# tcp-server.py

import select
import socket
import json
import datetime
import os
import logging
import sys

HOST = '10.179.40.11'
PORT = 48884
FASTLOADDIR = 'c:\\InSQL\\Data\\DataImport\\'
DATASIZE = 1024
#тэги historian которые создаются только для первого канала
ONLYONECHAN = 1
ONLYONE = ['in','T','rssi']

def get_non_blocking_server_socket(type):
    # Создаем сокет, который работает без блокирования основного потока
    server = socket.socket(socket.AF_INET, type)
    server.setblocking(False)

    # Биндим сервер на нужный адрес и порт
    server.bind((HOST, PORT))

    if (type == socket.SOCK_DGRAM):
        return server

    # Установка максимального количество подключений
    server.listen(10)

    return server


def parse_msg(msg):
    st = 0
    fi = 0
    rez = []
    while (st >=0 and fi >= 0):
        st = msg.find('{', fi)
        fi = msg.find('}', st)
        if (st >=0 and fi > 0):
            rez.append(json.loads(msg[st:fi+1]))
    return rez


# create csv data file
# {"id":"12.1","num":1,"dt":"2000-12-12 00:00:58","U":542,"R":299999,"Ub1":9.764,"Ub0":12.035,"U0":0,"in":1,"T":28.7,"rssi":-63}

def write_csv(js):
    
    dataname = ['dt', 'num', 'U', 'R', 'time', 'U0', 'Ub0', 'Ub1', 'T', 'rssi', 'in']
    for d in dataname:
        try:
            if js[d] == '':
                js[d] = ''
        except:
            js[d] = ''
            pass

    csvFilename = 'SODK_{}_{:%Y-%m}.csv'.format(js["id"], datetime.datetime.now())
    try:
        with open(csvFilename, 'r', newline='') as csvfile:
            n = csvfile.read(1)
    except:
        with open(csvFilename, 'w', newline='') as csvfile:
            csvfile.write('Datetime;Counter;U;R;time;U0;Ub0;Ub1;T;RSSI;in\n')
        pass
    
    try:
        with open(csvFilename, 'a', newline='') as csvfile:
            csvfile.write('{};{};{};{};{};{};{};{};{};{};{}\n'.format(js[str("dt")], js["num"], str(js["U"]).replace('.',','), str(js["R"]).replace('.',','), js["time"], str(js["U0"]).replace('.',','), str(js["Ub0"]).replace('.',','), str(js["Ub1"]).replace('.',','), str(js["T"]).replace('.',','), js["rssi"], js["in"]))
    except Exception as e:
        logging.error(e)
        pass


# create Wonderware Historian Fast load
def write_historian(js):
    id = js["id"]
    idn = id.split('.')
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
            if not ('T' in ONLYONE and ONLYONECHAN != idn[1]):
                f.write("Sodk_ZRTS{}_Tcpu,0,{},0,{},192\n".format(id, time_and_date, js["T"]))
            if not ('rssi' in ONLYONE and ONLYONECHAN != idn[1]):
                f.write("Sodk_ZRTS{}_rssi,0,{},0,{},192\n".format(id, time_and_date, js["rssi"]))
            if not ('in' in ONLYONE and ONLYONECHAN != idn[1]):
                f.write("Sodk_ZRTS{}_in,0,{},0,{},192\n".format(id, time_and_date, js["in"]))
    except Exception as e:
        #print('File error! {}'.format(e))
        logging.error(e)
        pass


if __name__ == '__main__':

    logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)-5.5s] %(message)s",
    handlers=[
        logging.FileHandler("debug.log"),
        logging.StreamHandler(sys.stdout)
    ]
)
    wonderware = False
    if os.path.isdir(FASTLOADDIR):
        wonderware = True
    else:
        logging.error("Wonderware Historian fastload disable.")
        #print("Wonderware Historian fastload disable.")
 
    # Откуда и куда записывать информацию
    outputs = []
    xinputs = []

    # Создаем серверный сокет без блокирования основного потока в ожидании подключения
    server_socket_tcp = get_non_blocking_server_socket(socket.SOCK_STREAM)
    server_socket_udp = get_non_blocking_server_socket(socket.SOCK_DGRAM)
    inputs = [server_socket_tcp, server_socket_udp]
    #print("TCP/UDP server is running")
    logging.info("TCP/UDP server is running")
    logging.debug("New: {}".format(str(server_socket_tcp)))
    logging.debug("New: {}".format(str(server_socket_udp)))

    while inputs:
        readables, writables, exceptional = select.select(inputs, outputs, xinputs)
        message = b''
        client_address = ('',0)
        #print("readadle len: " + str(len(readables)))
        for s in readables:
            #print("protocol: " + str(s.proto))
            if s == server_socket_tcp:
                connection, client_address = s.accept()
                connection.setblocking(False)
                inputs.append(connection)
                logging.debug("New: {}".format(str(s)))
                continue
            else: #UDP or connection
                #print("readadle: " + str(s))                
                logging.debug("Msg: {}".format(str(s)))
                try:
                    (message, client_address) = s.recvfrom(DATASIZE)
                    #print("readadle: " + str(s.type))
                    if s.type == socket.SOCK_STREAM:
                        client_address = s.getpeername()
                except Exception as e:
                    #print(e)
                    logging.error(e)
                    pass

            if message:
                # Вывод полученных данных на консоль
                #print("{}: {}".format(str(client_address), str(message)))
                logging.info("{}: {}".format(str(client_address), str(message)))
                try:
                    for js in parse_msg(message.decode(encoding="latin-1", errors="ignore")):
                        if js["id"]:
                            write_csv(js)
                            if wonderware:
                                write_historian(js)
                except Exception as e:
                    #print(e)
                    logging.error(e)
                    pass
            else:
                #print("close: " + str(s))
                if s not in [server_socket_tcp, server_socket_udp]:
                    inputs.remove(s)
                logging.debug("Cls: {}".format(str(s)))
                s.close()

