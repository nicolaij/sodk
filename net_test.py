# tcp-server.py

from datetime import datetime
from socket import *
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


csvFilename = 'SODK_{}.csv'.format('567.2')
with open(csvFilename, 'w', newline='') as csvfile:
    fieldnames = ['Datetime', 'last_name']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    writer.writeheader()
    writer.writerow({'first_name': 'Baked', 'last_name': 'Beans'})
    writer.writerow({'first_name': 'Lovely', 'last_name': 'Spam'})
    writer.writerow({'first_name': 'Wonderful', 'last_name': 'Spam'})

# the result is a JSON string:
print(s)

# Создать сокет сервера
server_socket = socket(AF_INET, SOCK_STREAM)
server_socket.connect((host, port))
cmd = 'GET http://pr4e.org/romeo.txt HTTP/1.0\r\n\r\n'.encode()
server_socket.send(cmd)
server_socket.close()
