#include "printf.hpp"

// Les fonctions printf() et printf_decimal() sont maintenant définies inline dans le header
// Ce fichier est conservé pour d'éventuelles fonctions futures

// Note: Les anciennes implémentations utilisaient huart2 et char_buff qui n'existent plus
// La nouvelle implémentation dans le header utilise huart1 et un buffer local

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
    SerialOut() : uart(&huart1) {}  // Utilisation de UART1
    
    void printf(const char* format, ...) {
        va_list args;
        va_start(args, format);
        int len = vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        
        if (len > 0) {
            HAL_UART_Transmit(uart, (uint8_t*)buffer, len, 100);
        }
    }
    
    void send(const char* message) {
        HAL_UART_Transmit(uart, (uint8_t*)message, strlen(message), 100);
    }
    
    // Fonction pour recevoir des données (bloquante)
    bool receive(uint8_t* data, uint16_t size, uint32_t timeout = 100) {
        return HAL_UART_Receive(uart, data, size, timeout) == HAL_OK;
    }
    
    // Fonction pour recevoir des données (non-bloquante avec interruption)
    bool receive_IT(uint8_t* data, uint16_t size) {
        return HAL_UART_Receive_IT(uart, data, size) == HAL_OK;
    }
};

#endif
