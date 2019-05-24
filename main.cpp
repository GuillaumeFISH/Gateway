#include "radio.h"

#if defined(SX127x_H)
    #define BW_KHZ              125
    #define SPREADING_FACTOR    7
    #define CF_HZ               915000000
#elif defined(SX126x_H)
    #define BW_KHZ              125
    #define SPREADING_FACTOR    7
    #define CF_HZ               915000000
#elif defined(SX128x_H)
    #define BW_KHZ              200
    #define SPREADING_FACTOR    7
    #define CF_HZ               2487000000
#endif

DigitalOut myled(LED1);
//Serial pc(PA_11,PA_12);
/**********************************************************************/

void txDoneCB()
{
}

void rxDoneCB(uint8_t size, float rssi, float snr)
{
    unsigned i;
    printf("%.1fdBm  snr:%.1fdB\t", rssi, snr);

    myled.write(!myled.read()); // toggle LED

    for (i = 0; i < size; i++) {
        printf("%d ", Radio::radio.rx_buf[i]);
    }
    printf("\r\n");
}

const RadioEvents_t rev = {
    /* Dio0_top_half */     NULL,
    /* TxDone_topHalf */    NULL,
    /* TxDone_botHalf */    txDoneCB,
    /* TxTimeout  */        NULL,
    /* RxDone  */           rxDoneCB,
    /* RxTimeout  */        NULL,
    /* RxError  */          NULL,
    /* FhssChangeChannel  */NULL,
    /* CadDone  */          NULL
};

int main()
{   
    printf("\r\nreset-rx\r\n");
    
    Radio::Init(&rev);

    //Radio::radio.hw_reset();
    Radio::Standby();
    Radio::LoRaModemConfig(BW_KHZ, SPREADING_FACTOR, 1);
    Radio::SetChannel(CF_HZ);

               // preambleLen, fixLen, crcOn, invIQ
    Radio::LoRaPacketConfig(8, false, true, false);

    Radio::Rx(0);
    
    for (;;) {     
        Radio::service();
    }
}

