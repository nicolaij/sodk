# tcp-server.py

import select
import socket
import sys
import json
import datetime

HOST = '10.179.40.11'
PORT = 48884
FASTLOADDIR = 'c:\\InSQL\\Data\\DataImport\\'
DATASIZE = 1024


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


def handle_readables(readables, server):
    # Обработка появления событий на входах
    for resource in readables:

        # Если событие исходит от серверного сокета, то мы получаем новое подключение
        if resource is server:
            connection, client_address = resource.accept()
            connection.setblocking(False)
            #INPUTS.append(connection)
            print("new connection from {address}".format(address=client_address))

        # Если событие исходит не от серверного сокета, но сработало прерывание на наполнение входного буффера
        else:
            data = ""
            try:
                data = resource.recv(1024)

            # Если сокет был закрыт на другой стороне
            except ConnectionResetError:
                pass

            if data:

                # Вывод полученных данных на консоль
                print("getting data: {data}".format(data=str(data)))

                # Говорим о том, что мы будем еще и писать в данный сокет
                #if resource not in OUTPUTS:
                #    OUTPUTS.append(resource)

            # Если данных нет, но событие сработало, то ОС нам отправляет флаг о полном прочтении ресурса и его закрытии
            else:

                # Очищаем данные о ресурсе и закрываем дескриптор
                clear_resource(resource)


def clear_resource(resource):
    """
    Метод очистки ресурсов использования сокета
    """
    if resource in OUTPUTS:
        OUTPUTS.remove(resource)
    if resource in INPUTS:
        INPUTS.remove(resource)
    resource.close()

    print('closing connection ' + str(resource))


def handle_writables(writables):
    # Данное событие возникает когда в буффере на запись освобождается место
    for resource in writables:
        try:
            resource.send(bytes('Hello from server!', encoding='UTF-8'))
        except OSError:
            clear_resource(resource)


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
# {"id":"12.1","num":1,"dt":"2000-12-12 00:00:58","U":542,"R":299999,"Ub1":9.764,"Ub0":12.035,"U0":0,"in":1,"T":28.7,"rssi":-63}

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
            csvfile.write('{},{},{},{},{},{},{},{},{},{}\n'.format(
                js["dt"], js["num"], js["U"], js["R"], js["U0"], js["Ub0"], js["Ub1"], js["T"], js["rssi"], js["in"]))
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
            f.write("Sodk_ZRTS{}_U,0,{},0,{},192\n".format(
                id, time_and_date, js["U"]))
            f.write("Sodk_ZRTS{}_R,0,{},0,{},192\n".format(
                id, time_and_date, js["R"]))
            f.write("Sodk_ZRTS{}_U0,0,{},0,{},192\n".format(
                id, time_and_date, js["U0"]))
            f.write("Sodk_ZRTS{}_UBatt0,0,{},0,{},192\n".format(
                id, time_and_date, js["Ub0"]))
            f.write("Sodk_ZRTS{}_UBatt1,0,{},0,{},192\n".format(
                id, time_and_date, js["Ub1"]))
            f.write("Sodk_ZRTS{}_Tcpu,0,{},0,{},192\n".format(
                id, time_and_date, js["T"]))
            f.write("Sodk_ZRTS{}_rssi,0,{},0,{},192\n".format(
                id, time_and_date, js["rssi"]))
            f.write("Sodk_ZRTS{}_in,0,{},0,{},192\n".format(
                id, time_and_date, js["in"]))
    except Exception as e:
        print('File error! {}'.format(e))
        pass


if __name__ == '__main__':
    # Откуда и куда записывать информацию
    outputs = list()
    xinputs = list()

    # Создаем серверный сокет без блокирования основного потока в ожидании подключения
    server_socket_tcp = get_non_blocking_server_socket(socket.SOCK_STREAM)
    server_socket_udp = get_non_blocking_server_socket(socket.SOCK_DGRAM)
    inputs = [server_socket_tcp, server_socket_udp]
    print("TCP/UDP server is running")

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
                continue
            else: #UDP or connection
                #print("readadle: " + str(s))                
                try:
                    (message, client_address) = s.recvfrom(DATASIZE)
                    #print("readadle: " + str(s.type))
                    if s.type == socket.SOCK_STREAM:
                        client_address = s.getpeername()
                except:
                    pass

            if message:
                try:
                    # Вывод полученных данных на консоль
                    print("{}: {}".format(str(client_address), str(message)))
                    for js in parse_msg(message.decode(encoding="latin-1", errors="ignore")):
                        #print("parsing: ", js)
                        if js:
                            write_csv(js)
                            #write_historian(js)
                except:
                    pass
            else:
                #print("close: " + str(s))
                s.close()
                if s not in [server_socket_tcp, server_socket_udp]:
                    inputs.remove(s)

