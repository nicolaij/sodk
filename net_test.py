# tcp-server.py

from datetime import datetime
import socket
import json
import csv

host = '10.179.40.11'
port = 48886

x = '{  "name": "John",  "age": 30,  "city": "New York"}'

m =  b'{"id":"12.1","num":1,"dt":"2000-12-12 00:00:58","U":542,"R":299999,"Ub1":9.764,"Ub0":12.035,"U0":0,"in":1,"T":28.7,"rssi":-63}{"id":"12.1","num":1,"dt":"2000-12-12 00:00:58","U":542,"R":299999,"Ub1":9.764,"Ub0":12.035,"U0":0,"in":1,"T":28.7,"rssi":-63}'

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
    while (st >=0 and fi > 0):
        st = msg.find('{', fi)
        fi = msg.find('}', st+1)
        if (st >=0 and fi > 0):
            rez.append(json.loads(msg[st:fi+1]))
    return rez

for js in parse_msg(m.decode(encoding="latin-1", errors="ignore")):
    print(js)
    print(js["Ub0"])
    print('{}'.format(js["Ub0"],","))

# convert into JSON:
j = json.loads(x)
dt = datetime.strptime("2022-10-05 11:39:15", "%Y-%m-%d %H:%M:%S")
print(dt)
s = j['name']


# Создать сокет сервера

MAX_CONNECTIONS = 20

clients = [socket.socket(socket.AF_INET, socket.SOCK_STREAM) for i in range(MAX_CONNECTIONS)]
udp_cli = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for client in clients:
    client.connect((host, port))

for i in range(MAX_CONNECTIONS):
    clients[i].send(m)
    udp_cli.sendto(m, (host, port))
