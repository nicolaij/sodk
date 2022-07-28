# tcp-server.py

from socket import *

host = '10.179.40.11'
port = 48884

# Создать сокет сервера
server_socket = socket(AF_INET, SOCK_DGRAM)
cmd = 'GET http://pr4e.org/romeo.txt HTTP/1.0\r\n\r\n'.encode()
server_socket.sendto(cmd,(host, port))
