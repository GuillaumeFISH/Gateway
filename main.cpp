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

/* Defines the two queues used, one for events and one for printing to the screen */
EventQueue TimeQueue;

/* Defines the timer */
Timer t;
time_t whattime;
int64_t usTime1 = 0, usTime2 = 0, usDeltaTime = 0;

//Updates the time at an interval defined by the read ticker
void Update_time() {
  // this runs in the normal priority thread
    usTime2 = usTime1;
    usTime1 = t.read_high_resolution_us();
    usDeltaTime = usTime1 - usTime2;
    whattime = time(NULL);
}


void txDoneCB()
{
}

void rxDoneCB(uint8_t size, float rssi, float snr)
{
    printf("-----------------------------\r\n");
    printf("\r\nCurrent RX Epoch Time: %u\r\n", (unsigned int)whattime);
    unsigned i;
    printf("RSSI: %.1fdBm  SNR: %.1fdB\r\n", rssi, snr);

    int sent_time = (int)(Radio::radio.rx_buf[0] << 24 | Radio::radio.rx_buf[1] << 16 | Radio::radio.rx_buf[2] << 8 | Radio::radio.rx_buf[3]);
    printf("Received time: %d\a\r\n", sent_time);
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
    
    /* resets and starts the timer */
    t.reset();
    t.start();
    usTime1 = t.read_high_resolution_us();

    //Initialize receiver settings
    Radio::Init(&rev);
    Radio::Standby();
    Radio::LoRaModemConfig(BW_KHZ, SPREADING_FACTOR, 1);
    Radio::SetChannel(CF_HZ);
               // preambleLen, fixLen, crcOn, invIQ
    Radio::LoRaPacketConfig(8, false, true, false);

    //Tells the receiver to listen forever
    Radio::Rx(0);
    
    // normal priority thread for other events
    Thread eventThread(osPriorityNormal);
    eventThread.start(callback(&TimeQueue, &EventQueue::dispatch_forever));

    //call read_sensors 1 every second, automatically defering to the eventThread
    Ticker ReadTicker;
    ReadTicker.attach(TimeQueue.event(&Update_time), 3.0f);
    //not using printticker for now
    //PrintTicker.attach(printfQueue.event(&Print_Sensors), 3.0f);

    //This services interrupts. No idea how it works, how often it should be called etc.
    //but it works for now i guess
    for (;;) {     
        Radio::service();
    }

    wait(osWaitForever);
}

