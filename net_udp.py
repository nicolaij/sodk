# tcp-server.py

from socket import *

host = '10.179.40.11'
port = 48884
bufferSize  = 1024

# Создать сокет сервера
server_socket = socket(AF_INET, SOCK_DGRAM)

#   s et et
server_addr = (host, port)
server_socket.bind(server_addr)

# Обработка запроса подключения
try:
    while True:
        bytesAddressPair = server_socket.recvfrom(bufferSize)

        message = bytesAddressPair[0]
        address = bytesAddressPair[1]

        clientMsg = "Message from Client:{}".format(message)
        clientIP  = "Client IP Address:{}".format(address)
        
        print(clientMsg)
        print(clientIP)

except:
    server_socket.close()
    print("socket closed.")
