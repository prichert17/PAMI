#ifndef PRINTF_HPP
#define PRINTF_HPP

#include "usart.h"
#include <cstdarg>
#include <cstdio>
#include <cstring>

class SerialOut {
private:
    UART_HandleTypeDef* uart;
    char buffer[256];
    
public:
    SerialOut() : uart(&huart1) {}  // Utilisation de UART1 au lieu de UART2
    
    void printf(const char* format, ...) {
        va_list args;
        va_start(args, format);
        int len = vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        
        if (len > 0) {
            HAL_UART_Transmit(uart, (uint8_t*)buffer, len, 100);
        }
    }
    
    void printf_decimal(double value, int8_t precision) {
        char format[20];
        snprintf(format, sizeof(format), "%%.%df\t", precision);
        snprintf(buffer, sizeof(buffer), format, value);
        HAL_UART_Transmit(uart, (uint8_t*)buffer, strlen(buffer), 100);
    }
    
    void send(const char* message) {
        HAL_UART_Transmit(uart, (uint8_t*)message, strlen(message), 100);
    }
};

#endif