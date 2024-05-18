# tcp-server.py

import uno
import select
import socket
import json
import datetime
import os
import logging
import sys
import websocket
import asyncio
import time

HOST = '10.179.40.11'
PORT = 48886
FASTLOADDIR = 'c:\\InSQL\\Data\\DataImport\\'
DATASIZE = 1024

dataname = ['id', 'dt', 'num', 'U', 'R', 'time', 'U0', 'Ulv', 'Rlv', 'U0lv', 'Ub0', 'Ub1', 'T', 'rssi', 'in', 'T2', 'H2']

officedataname = ['dt', 'num','U','R', 'U0', 'Ub1']

# soffice "-accept=socket,host=localhost,port=2002;urp;"

openoffice = True
desktop = None
o_sheets = None
odoc = None
dEndRow = [0] * 10


def create_office_instance():
    localContext = uno.getComponentContext()
    resolver = localContext.ServiceManager.createInstanceWithContext(
        "com.sun.star.bridge.UnoUrlResolver", localContext)
    ctx = resolver.resolve(
        "uno:socket,host=localhost,port=2002;urp;StarOffice.ComponentContext")
    smgr = ctx.ServiceManager
    desktop = smgr.createInstanceWithContext("com.sun.star.frame.Desktop", ctx)
    return desktop


def open_sheets(doc):
    if doc is None:
        doc = desktop.loadComponentFromURL("private:factory/scalc", "_blank", 0, ())
    
    for chan in range(1,9):
        try:
            csheet = doc.Sheets[chan]
        except:
            csheet = doc.Sheets.insertNewByName("data" + str(chan), chan)

    csheet = doc.Sheets[1]
    doc.CurrentController.setActiveSheet(csheet)
    return doc.Sheets


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
    while (st >= 0 and fi >= 0):
        st = msg.find('{', fi)
        fi = msg.find('}', st)
        if (st >= 0 and fi > 0):
            rez.append(json.loads(msg[st:fi+1]))
    return rez


# create csv data file
# {"id":"12.1","num":1,"dt":"2000-12-12 00:00:58","U":542,"R":299999,"Ub1":9.764,"Ub0":12.035,"U0":0,"in":1,"T":28.7,"rssi":-63}

def write_csv(prename, js):

    for d in dataname:
        if d not in js:
            js[d] = ''

    csvFilename = '{}{}_{:%Y-%m}.csv'.format(prename, js["id"], datetime.datetime.now())
    try:
        with open(csvFilename, 'r', newline='') as csvfile:
            n = csvfile.read(1)
    except:
        with open(csvFilename, 'w', newline='') as csvfile:
            csvfile.write('Datetime;Counter;U;R;time;U0;Ulv;Rlv;U0lv;Ub0;Ub1;T;RSSI;in;T2;H2\n')
        pass

    try:
        with open(csvFilename, 'a', newline='') as csvfile:
            csvfile.write('{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{}\n'.format(js[str("dt")], js["num"], str(js["U"]).replace('.', ','), str(js["R"]).replace('.', ','), js["time"], str(js["U0"]).replace('.', ','), str(js["Ulv"]).replace('.', ','), str(
                js["Rlv"]).replace('.', ','), str(js["U0lv"]).replace('.', ','), str(js["Ub0"]).replace('.', ','), str(js["Ub1"]).replace('.', ','), str(js["T"]).replace('.', ','), js["rssi"], js["in"], str(js["T2"]).replace('.', ','), str(js["H2"]).replace('.', ',')))
    except Exception as e:
        logging.error(e)
        pass


# create Wonderware Historian Fast load
def write_historian(js):
    id = js["id"]
    idn = id.split('.')
    chan = int(idn[1], base=10)
    IntouchFilename = '{}{} {:%Y-%m-%d_%H.%M.%S.%f}.csv'.format(FASTLOADDIR, id, datetime.datetime.now())
    try:
        with open(IntouchFilename, 'w', encoding="latin-1") as f:
            f.write("ASCII\n,\n")
            f.write("SODK,1,Server Local,1,1\n")
            dt = datetime.datetime.strptime(js["dt"], "%Y-%m-%d %H:%M:%S")
            time_and_date = '{:%Y/%m/%d,%H:%M:%S}.000'.format(dt)
            if (("U" in js) and (js["U"] != '')):
                f.write("Sodk_ZRTS{}_U,0,{},0,{},192\n".format(
                    id, time_and_date, js["U"]))
            if (("R" in js) and (js["R"] != '')):
                f.write("Sodk_ZRTS{}_R,0,{},0,{},192\n".format(
                    id, time_and_date, js["R"]))
            if (("U0" in js) and (js["U0"] != '')):
                f.write("Sodk_ZRTS{}_U0,0,{},0,{},192\n".format(
                    id, time_and_date, js["U0"]))
            if (("Ulv" in js) and (js["Ulv"] != '')):
                f.write("Sodk_ZRTS{}_Ulv,0,{},0,{},192\n".format(
                    id, time_and_date, js["Ulv"]))
            if (("Rlv" in js) and (js["Rlv"] != '')):
                f.write("Sodk_ZRTS{}_Rlv,0,{},0,{},192\n".format(
                    id, time_and_date, js["Rlv"]))
            if (("U0lv" in js) and (js["U0lv"] != '')):
                f.write("Sodk_ZRTS{}_U0lv,0,{},0,{},192\n".format(
                    id, time_and_date, js["U0lv"]))
            if (("Ub0" in js) and (js["Ub0"] != '')):
                f.write("Sodk_ZRTS{}_UBatt0,0,{},0,{},192\n".format(
                    id, time_and_date, js["Ub0"]))
            if (("Ub1" in js) and (js["Ub1"] != '')):
                f.write("Sodk_ZRTS{}_UBatt1,0,{},0,{},192\n".format(
                    id, time_and_date, js["Ub1"]))
            if (("T" in js) and (js["T"] != '')):
                f.write("Sodk_ZRTS{}.1_Tcpu,0,{},0,{},192\n".format(
                    idn[0], time_and_date, js["T"]))
            if (("T2" in js) and (js["T2"] != '')):
                f.write("Sodk_ZRTS{}.1_T2,0,{},0,{},192\n".format(
                    idn[0], time_and_date, js["T2"]))
            if (("H2" in js) and (js["H2"] != '')):
                f.write("Sodk_ZRTS{}.1_H2,0,{},0,{},192\n".format(
                    idn[0], time_and_date, js["H2"]))
            if (("rssi" in js) and (js["rssi"] != '')):
                f.write("Sodk_ZRTS{}.1_rssi,0,{},0,{},192\n".format(
                    idn[0], time_and_date, js["rssi"]))
            if (("in" in js) and (js["in"] != '')):
                f.write("Sodk_ZRTS{}.1_in,0,{},0,{},192\n".format(
                    idn[0], time_and_date, js["in"]))
    except Exception as e:
        # print('File error! {}'.format(e))
        logging.error(e)
        pass

def on_wsopen(wsapp):
    wsapp.send("openws:" + str(time.time()))

def on_wsmessage(wsapp, message):
    for js in parse_msg(message):
        if "channel" in js:
            if openoffice:
                chan = int(js['channel'])
                num = 0
                for d in officedataname:
                    try:
                        if d == 'dt':
                            o_sheets[chan].getCellByPosition(num, dEndRow[chan] + 1).Formula = js[d]
                            numbers = odoc.NumberFormats
                            locale = odoc.CharLocale                                            
                            NumberFormatString = "YYYY-MM-DD HH:MM:SS"
                            number_format_key = numbers.queryKey(NumberFormatString, locale, True)
                            if number_format_key == -1:
                                number_format_key = numbers.addNew(NumberFormatString, locale)
                            o_sheets[chan].getCellByPosition(num, dEndRow[chan] + 1).NumberFormat = number_format_key
                        else:
                            o_sheets[chan].getCellByPosition(num, dEndRow[chan] + 1).Value = js[d]
                    except:
                        pass
                    num = num + 1
                dEndRow[chan] = dEndRow[chan] + 1

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
        # print("Wonderware Historian fastload disable.")

    try:
        desktop = create_office_instance()
    except:
        openoffice = False
        logging.error("OpenOffice disable.")

    if openoffice:
        logging.info("OpenOffice connect succefull.")
        odoc = desktop.getCurrentComponent()
        o_sheets = open_sheets(odoc)
        for chan in range(1,9):
            oCurs = o_sheets[chan].createCursor()
            oCurs.gotoEndOfUsedArea(False)
            dEndRow[chan] = oCurs.RangeAddress.EndRow
            if dEndRow[chan] == 0:
                num = 0
                for d in officedataname:
                    try:
                        o_sheets[chan].getCellByPosition(num, dEndRow[chan]).String = str(d)
                    except:
                        pass
                    num = num + 1

        os.environ['NO_PROXY'] = '192.168.4.1'
        wsapp = websocket.WebSocketApp("ws://192.168.4.1/ws", on_open=on_wsopen, on_message=on_wsmessage)
        while True:
            try:
                wsapp.run_forever()
            except Exception as e:
                logging.error(e)
            
            time.sleep(1)

    # Откуда и куда записывать информацию
    outputs = []
    xinputs = []

    # Создаем серверный сокет без блокирования основного потока в ожидании подключения
    server_socket_tcp = get_non_blocking_server_socket(socket.SOCK_STREAM)
    server_socket_udp = get_non_blocking_server_socket(socket.SOCK_DGRAM)
    
    inputs = [server_socket_tcp, server_socket_udp]
    # print("TCP/UDP server is running")
    logging.info("TCP/UDP server is running")
    logging.debug("New: {}".format(str(server_socket_tcp)))
    logging.debug("New: {}".format(str(server_socket_udp)))

    while inputs:
        readables, writables, exceptional = select.select(inputs, outputs, xinputs)
        message = b''
        client_address = ('', 0)
        # print("readadle len: " + str(len(readables)))
        for s in readables:
            # print("protocol: " + str(s.proto))
            if s == server_socket_tcp:
                connection, client_address = s.accept()
                connection.setblocking(False)
                inputs.append(connection)
                logging.debug("New: {}".format(str(s)))
                continue
            else:  # UDP or connection
                # print("readadle: " + str(s))
                logging.debug("Msg: {}".format(str(s)))
                try:
                    (message, client_address) = s.recvfrom(DATASIZE)
                    # print("readadle: " + str(s.type))
                    if s.type == socket.SOCK_STREAM:
                        client_address = s.getpeername()
                except Exception as e:
                    # print(e)
                    logging.error(e)
                    pass

            if message:
                # Вывод полученных данных на консоль
                # print("{}: {}".format(str(client_address), str(message)))
                logging.info("{}: {}".format(str(client_address), str(message)))
                try:
                    for js in parse_msg(message.decode(encoding="latin-1", errors="ignore")):
                        if "id" in js:
                            write_csv("SODK_", js)
                            if wonderware:
                                write_historian(js)
                        else:
                            if "channel" in js:
                                if openoffice:
                                    chan = int(js['channel'])
                                    num = 0
                                    for d in officedataname:
                                        try:
                                            if d == 'dt':
                                                o_sheets[chan].getCellByPosition(num, dEndRow[chan] + 1).Formula = js[d]
                                                numbers = odoc.NumberFormats
                                                locale = odoc.CharLocale                                            
                                                NumberFormatString = "YYYY-MM-DD HH:MM:SS"
                                                number_format_key = numbers.queryKey(NumberFormatString, locale, True)
                                                if number_format_key == -1:
                                                    number_format_key = numbers.addNew(NumberFormatString, locale)
                                                o_sheets[chan].getCellByPosition(num, dEndRow[chan] + 1).NumberFormat = number_format_key
                                            else:
                                                o_sheets[chan].getCellByPosition(num, dEndRow[chan] + 1).Value = js[d]
                                            #print(o_sheets[chan].getCellByPosition(num, dEndRow[chan] + 1).Value)
                                        except:
                                            pass
                                        num = num + 1
                                    dEndRow[chan] = dEndRow[chan] + 1

                except Exception as e:
                    # print(e)
                    logging.error(e)
                    pass
            else:
                # print("close: " + str(s))
                if s not in [server_socket_tcp, server_socket_udp]:
                    inputs.remove(s)
                logging.debug("Cls: {}".format(str(s)))
                s.close()
