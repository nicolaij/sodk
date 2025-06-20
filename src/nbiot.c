esp_err_t at_csosend(int socket, char *data, char *buffer, int len_data)
    char buf[20];
    int l = snprintf(buf, sizeof(buf), "AT+CSOSEND=%d,%d,", socket, len_data * 2);
    int txBytes = uart_write_bytes(UART_NUM_1, buf, l);
