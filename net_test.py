# tcp-server.py

from datetime import datetime
import socket
import json
import csv
import psycopg2
import re

host = '10.179.40.11'
port = 48886

x = '{  "name": "John",  "age": 30,  "city": "New York"}'

m = b'{"id":"sodk1.10","num":1,"dt":"2025-06-24 13:13:34","U":11544,"R":5999,"U0":8378,"Ubatt1":9375,"time":63,"NBbatt":3.373,"RSSI":-65,"Temp":44.7,"Flags":"0x0000"}'


def parse_msg1(msg):
    try:
        return [json.loads(msg)]
    except json.JSONDecodeError as err:
        if err.msg == 'Extra data':
            head = [json.loads(msg[0:err.pos])]
            tail = parse_msg1(msg[err.pos:])
            return head + tail
        else:
            return []


def parse_msg(msg):
    st = 0
    fi = 1
    rez = []
    while (st >= 0 and fi > 0):
        st = msg.find('{', fi)
        fi = msg.find('}', st+1)
        if (st >= 0 and fi > 0):
            rez.append(json.loads(msg[st:fi+1]))
    return rez


def add_to_postgre(j):
    try:
        # пытаемся подключиться к базе данных
        conn = psycopg2.connect(dbname='history', user='sodk', password='sodk', host='10.179.40.3')
    except:
        # в случае сбоя подключения будет выведено сообщение в STDOUT
        print('Can`t establish connection to database')
    id = re.findall(r'^([A-Za-z]*)(\d+)\.?(\d*)', j['id'])[0]
    table = id[0]
    idn = id[1]
    channel = id[2]
    print("INSERT INTO sodk (id, channel, dt, num, U) VALUES (%s, %s, %s, %s, %s);", (int(idn), int(channel), j["dt"], j["num"], j["U"]))

    if (table == "sodk"):
        with conn.cursor() as curs:
            curs.execute("INSERT INTO sodk (id, channel, dt, num, U) VALUES (%s, %s, %s, %s, %s);", (int(idn), int(channel), j["dt"], j["num"], j["U"]))
            #curs.execute("INSERT INTO sodk (id, channel, dt, U, R, U0, Ubatt1, time, RSSI, Temp, Flags) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s );", (str(idn), str(channel), str(j["dt"]), str(j["U"]), j["R"], j["U0"], j["Ubatt1"], j["time"], j["NBbatt"], j["RSSI"], j["Temp"], j["Flags"]))

    with conn.cursor() as curs:
        curs.execute('SELECT * FROM sodk')
        rows = curs.fetchall()
        print(rows)
    
    conn.commit()  
    conn.close()  # закрываем соединение


# for js in parse_msg(m.decode(encoding="latin-1", errors="ignore")):
js = json.loads(m)
print(js)
add_to_postgre(js)

# convert into JSON:
j = json.loads(m)
dt = datetime.strptime("2022-10-05 11:39:15", "%Y-%m-%d %H:%M:%S")
print(dt)
exit()
# Создать сокет сервера

MAX_CONNECTIONS = 20

clients = [socket.socket(socket.AF_INET, socket.SOCK_STREAM) for i in range(MAX_CONNECTIONS)]
udp_cli = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for client in clients:
    client.connect((host, port))

for i in range(MAX_CONNECTIONS):
    clients[i].send(m)
    udp_cli.sendto(m, (host, port))
