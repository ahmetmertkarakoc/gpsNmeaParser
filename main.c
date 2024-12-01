#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#define BUFFER_SIZE 256
#define NMEA_MAX_LENGTH 256

typedef struct {
    float latitude;
    float longitude;
    float altitude;
} GPS_info;

GPS_info gpsData;


typedef struct {
    uint8_t buffer[BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuffer;

// Ring buffer tanımları
RingBuffer uart0Buffer; // PC <-> Tiva
RingBuffer uart1Buffer; // Mosaic-X5 <-> Tiva

// Ring buffer fonksiyonları
void RingBuffer_Init(RingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
}

bool RingBuffer_Write(RingBuffer *rb, uint8_t data) {
    if (((rb->head + 1) % BUFFER_SIZE) == rb->tail) {
        return false; // Buffer dolu
    }
    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % BUFFER_SIZE;
    return true;
}

bool RingBuffer_Read(RingBuffer *rb, uint8_t *data) {
    if (rb->head == rb->tail) {
        return false; // Buffer boş
    }
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % BUFFER_SIZE;
    return true;
}

// UART kesme işleyicileri
void UART0IntHandler(void) {
    uint32_t status = ROM_UARTIntStatus(UART0_BASE, true);
    ROM_UARTIntClear(UART0_BASE, status);

    while (ROM_UARTCharsAvail(UART0_BASE)) {
        uint8_t receivedChar = ROM_UARTCharGetNonBlocking(UART0_BASE);
        if (!RingBuffer_Write(&uart0Buffer, receivedChar)) {
            UARTSend(UART0_BASE, (uint8_t *)"UART0 Buffer Full\n", 18);
        }
    }
}

void UART1IntHandler(void) {
    uint32_t status = ROM_UARTIntStatus(UART1_BASE, true);
    ROM_UARTIntClear(UART1_BASE, status);

    while (ROM_UARTCharsAvail(UART1_BASE)) {
        uint8_t receivedChar = ROM_UARTCharGetNonBlocking(UART1_BASE);
        if (!RingBuffer_Write(&uart1Buffer, receivedChar)) {
            UARTSend(UART0_BASE, (uint8_t *)"UART1 Buffer Full\n", 18);
        }
    }
}

// UART üzerinden veri gönderme
void UARTSend(uint32_t base, const uint8_t *data, uint32_t count) {
    while (count--) {
        ROM_UARTCharPut(base, *data++);
    }
}

void ParseGPGGA(const char *nmeaSentence) {
    char type[6];
    char lat[10], ns, lon[11], ew;
    float altitude;

    // NMEA ayrıştırma
    if (sscanf(nmeaSentence, "$%5s,%*f,%9[^,],%c,%10[^,],%c,%*d,%*d,%*f,%f", type, lat, &ns, lon, &ew, &altitude) == 6) {
        // Latitude ve Longitude işaretlerine göre dönüştür
        gpsData.latitude = atof(lat) / 100.0f; // Derece formatına dönüştürme
        if (ns == 'S') gpsData.latitude = -gpsData.latitude;

        gpsData.longitude = atof(lon) / 100.0f; // Derece formatına dönüştürme
        if (ew == 'W') gpsData.longitude = -gpsData.longitude;

        gpsData.altitude = altitude;

        // Debug veya UART ile yazdırma
        char debugMsg[128];
        snprintf(debugMsg, sizeof(debugMsg), "Lat: %.6f, Lon: %.6f, Alt: %.2f\n",
                 gpsData.latitude, gpsData.longitude, gpsData.altitude);
        UARTSend(UART0_BASE, (uint8_t *)debugMsg, strlen(debugMsg));
    } else {
        UARTSend(UART0_BASE, (uint8_t *)"Invalid GPGGA sentence\n", 23);
    }
}

// NMEA cümlesini işleme
void ProcessNMEA(const char *nmeaSentence) {
    UARTSend(UART0_BASE, (uint8_t *)nmeaSentence, strlen(nmeaSentence));
    UARTSend(UART0_BASE, (uint8_t *)"\n", 1);

    if (strncmp(nmeaSentence, "$GPGGA", 6) == 0) {
        ParseGPGGA(nmeaSentence); // GPGGA cümlesini ayrıştır
    }
}

// NMEA cümlelerini ring buffer'dan kontrol etme
void CheckForNMEASentences() {
    static char nmeaBuffer[NMEA_MAX_LENGTH];
    static uint8_t nmeaIndex = 0;

    uint8_t data;
    while (RingBuffer_Read(&uart1Buffer, &data)) {
        if (data == '\n' || data == '\r') { // Satır sonu
            if (nmeaIndex > 0) { // Cümle tamamlandı
                nmeaBuffer[nmeaIndex] = '\0'; // String sonlandır
                ProcessNMEA(nmeaBuffer); // Cümleyi işle
                nmeaIndex = 0; // Index sıfırla
            }
        } else if (nmeaIndex < sizeof(nmeaBuffer) - 1) {
            nmeaBuffer[nmeaIndex++] = data;
        } else {
            // Buffer overflow
            UARTSend(UART0_BASE, (uint8_t *)"NMEA Buffer Overflow\n", 22);
            nmeaIndex = 0; // Resetle
        }
    }
}

// Ana fonksiyon
int main(void) {
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // UART0 (PC haberleşmesi)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    ROM_IntEnable(INT_UART0);

    // UART1 (Mosaic-X5 haberleşmesi)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
    ROM_IntEnable(INT_UART1);

    ROM_IntMasterEnable();
    RingBuffer_Init(&uart0Buffer);
    RingBuffer_Init(&uart1Buffer);

    UARTSend(UART0_BASE, (uint8_t *)"Uartlar Düzgün Çalışıyor \n", 30);

    while (1) {
        CheckForNMEASentences();
    }
}
