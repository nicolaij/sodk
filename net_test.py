# tcp-server.py

from socket import *

host = '10.179.40.11'
port = 48884

# Создать сокет сервера
server_socket = socket(AF_INET, SOCK_STREAM)
server_socket.connect((host, port))
cmd = 'GET http://pr4e.org/romeo.txt HTTP/1.0\r\n\r\n'.encode()
server_socket.send(cmd)
server_socket.close()
