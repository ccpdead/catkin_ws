#include "jhr_usart.h"

serial::Serial Robot_Serial;

Jhr_usart::Jhr_usart() {}
Jhr_usart::~Jhr_usart() {}
void Jhr_usart::usart_init() {
    try {
        Robot_Serial.setPort("/dev/ttyCH9344USB1");
        Robot_Serial.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        Robot_Serial.setTimeout(to);
        Robot_Serial.open();
    } catch (serial::IOException& e) {
        perror("Unable open usart.................");
    }
    if (Robot_Serial.isOpen()) {
        perror("Serial Opened.....................");
    }
}
void Jhr_usart::close() {
    Robot_Serial.close();
}

bool Jhr_usart::ReadFromUsart(const char* Buffer, int size) {
    Robot_Serial.read((unsigned char*)Buffer, size);
    printf("read:");
    for (int i = 0; i < size; i++) {
        printf("%x ", Buffer[i]);
    }
    printf("\n");
    return true;
}

void Jhr_usart::WriteToUsart(const char* Buffer, int size) {
    Robot_Serial.write((unsigned char*)Buffer, size);
    printf("write:");
    for (int i = 0; i < size; i++) {
        printf("%x ", Buffer[i]);
    }
    printf("\n");
}