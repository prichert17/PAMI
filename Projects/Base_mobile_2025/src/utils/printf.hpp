#ifndef __PRINTF__
#define __PRINTF__

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ST_files/usart.h"
#include "constants.hpp"

/*
    This class is used to print data to the serial port
*/
class SerialOut{
    private:
        char char_buff[CONSTANTS::PRINTF_BUFFER_SIZE];  // Buffer to use for serial output
        const uint32_t buff_size = CONSTANTS::PRINTF_BUFFER_SIZE;   // Size of the buffer

    public:
        /**
         * @brief Construct a new SerialOut object
         */
        SerialOut();

        /**
         * @brief Print a formatted string to the serial port
         * @param format Format string
         * @param ... Arguments to format
         * @warning !!! Seems to be broken for all numbers !!!
         */
        void printf(const char *format, ...);

        /**
         * @brief Print a decimal number to the serial port with a given precision
         * @param _x Number to print
         */
        void printf_decimal(double _x, int8_t precision = 9);
};

#endif