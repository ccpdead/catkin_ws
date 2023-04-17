#ifndef __JHR_USART_
#define __JHR_USART_

#include <serial/serial.h>
#include <stdio.h>
#include <stdlib.h>

class Jhr_usart {
   public:
    Jhr_usart();
    ~Jhr_usart();
    static void usart_init();
    static void close();
    static bool ReadFromUsart(const char* Buffer,int size);
    static void WriteToUsart(const char* Buffer,int size);
};

#endif