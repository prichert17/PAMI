#include "printf.hpp"


SerialOut::SerialOut(){
    memset(char_buff, '\0', sizeof(char_buff)); // Initialize buffer to 0
}

void SerialOut::printf(const char *format, ...){
    memset(char_buff, '\0', sizeof(char_buff)); // TODO: somehow remove this to improve performance
    int32_t len = snprintf(char_buff, buff_size, format);  // Format the string
    // If the string is too big to print, print an error message and return
    if(len < 0 || len > buff_size){
        snprintf(char_buff, buff_size, "[ERROR] String too big to print!");
        return;   
    }
    // Print the formatted string
    HAL_UART_Transmit(&huart2, (uint8_t*)char_buff, sizeof(char_buff), 100);
}

void SerialOut::printf_decimal(double _x, int8_t precision){
    // TODO : check inputs
    unsigned long int a,b;  // Integer and decimal part of the number
    double m;    // 
    double x = _x;   // Copy of the number to print
    char expo = ' ';    // Exponent of the number
    int64_t mult = pow(10, precision);  // Multiplier to get the decimal part
    char s = '+';   // Sign of the number

    // Find the sign of the number
    if(x < 0.0){
        s = '-';
        x = -x;    
    }

    // Multiply the number by a power of 1000 based on its value
    if(x>1.0){
        x = x;
    }
    else if(x<1.0 && x>=1e-3){
        x = x*1e3;
        expo = 'm'; 
    }
    else if(x<1e-3 && x>=1e-6){
        x = x*1e6;
        expo = 'u';
    }
    else if(x<1e-6 && x>=1e-9){
        x = x*1e9;
        expo = 'n';
    }
    else if(x<1e-9 && x>=1e-12){
        x = x*1e12;
        expo = 'p';
    }
    else if(x<1e-12 && x>=1e-15){
        x = x*1e15;
        expo = 'f';
    }
    else{
        memset(char_buff, '\0', sizeof(char_buff)); // TODO: somehow remove this to improve performance
        uint32_t len = snprintf(char_buff, buff_size, "[ERROR] Number too small to print!");    // Format the string
        HAL_UART_Transmit(&huart2, (uint8_t*)char_buff, sizeof(char)*len, 100);     // Print the formatted string
        return;
    }

    m = x*mult; // Multiply by the multiplier
    a = (int)x; // Get the integer part
    b = (int)(m-a*mult);    // Get the decimal part
    
    memset(char_buff, '\0', sizeof(char_buff)); // TODO: somehow remove this to improve performance
    uint32_t len = snprintf(char_buff, buff_size, "%c%lu.%lu%c",s,a,b,expo);    // Format the string
    HAL_UART_Transmit(&huart2, (uint8_t*)char_buff, sizeof(char)*len, 100);     // Print the formatted string
}
