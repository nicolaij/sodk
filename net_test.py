# tcp-server.py

from datetime import datetime
import socket
import json
import csv

host = '10.179.40.11'
port = 48884

x = '{  "name": "John",  "age": 30,  "city": "New York"}'

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
    #clients[i].send(bytes("hello from client number " + str(i), encoding='UTF-8'))
    udp_cli.sendto(bytes("hello from UDP client number " + str(i), encoding='UTF-8'), (host, port))

for client in clients:
    data = client.recv(1024)
    print(str(data))
