# tcp-server.py

from socket import *

host = '10.179.40.11'
port = 48884

# Создать сокет сервера
server_socket = socket(AF_INET, SOCK_STREAM)

#   s et et
server_addr = (host, port)
server_socket.bind(server_addr)

# Начать прослушивание, максимально допустимое соединение № 5
server_socket.listen(5)

# Обработка запроса подключения
try:
    while True:
        print('waiting for connect...')
        #   В ожидании подключения клиента
        client_socket, client_addr = server_socket.accept()
        # После успешного соединения печати информации клиента
        print('a client connnect from:', client_addr)
        while(True):
            # Отправить данные клиенту
            # client_socket.send('Hello, client!'.encode())
            # Получите данные клиента
            data = client_socket.recv(1024)
            if (len(data)) < 1:
                client_socket.close()
                print("client socket closed.")
                break

            print('recv data is: \"%s\"'%data.decode())

            # Получить выйти, выключить розетку
            # if "quit" in data.decode():
            #    break

except:
    client_socket.close()
    server_socket.close()
    print("socket closed.")
