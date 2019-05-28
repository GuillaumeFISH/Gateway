#include "radio.h"

#if defined(SX127x_H)
    #define BW_KHZ              125
    #define SPREADING_FACTOR    7
    #define CF_HZ               915000000
    #define TX_DBM              20
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
Serial pc(PA_11,PA_12);
/**********************************************************************/
volatile bool txDone;

/* Defines the two queues used, one for events and one for printing to the screen */
EventQueue TimeQueue;
EventQueue TransmitQueue;

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
    txDone = true;
    printf("Query sent\r\n");
}

void rxDoneCB(uint8_t size, float rssi, float snr)
{
    printf("------RECEIVING PACKET------\r\n");
    //printf("\r\nCurrent RX Epoch Time: %u\r\n", (unsigned int)whattime);
    printf("RSSI: %.1fdBm  SNR: %.1fdB\r\n", rssi, snr);

    //This is really wonky I know but i cant be bothered to write a function
    int sent_time = (int)(Radio::radio.rx_buf[0] << 24 | Radio::radio.rx_buf[1] << 16 | Radio::radio.rx_buf[2] << 8 | Radio::radio.rx_buf[3]);
    int sent_temp1 = (int)(Radio::radio.rx_buf[4] << 24 | Radio::radio.rx_buf[5] << 16 | Radio::radio.rx_buf[6] << 8 | Radio::radio.rx_buf[7]);
    float fl_temp1 = ((float)sent_temp1)/100;
    int sent_accz = (int)(Radio::radio.rx_buf[8] << 24 | Radio::radio.rx_buf[9] << 16 | Radio::radio.rx_buf[10] << 8 | Radio::radio.rx_buf[11]);
    printf("Received time: %d\a\r\n", sent_time);
    printf("Received temp: %.2f\r\n", fl_temp1);
    printf("Received accz: %d\r\n", sent_accz);
    printf("------PACKET  RECEIVED------\r\n\r\n");
}

void Send_transmission() {
    //Building the payload
    Radio::radio.tx_buf[0] = 0xAA;
    pc.printf("Sending Query\r\n");

    txDone = false;
    Radio::Send(1, 0, 0, 0);   /* begin transmission */
    while (!txDone) {
        Radio::service();
    }
    printf("Done servicing\r\n\r\n");
    Radio::Rx(0);
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
    Radio::set_tx_dbm(TX_DBM);
               // preambleLen, fixLen, crcOn, invIQ
    Radio::LoRaPacketConfig(8, false, true, false);
    
    // normal priority thread for other events
    Thread eventThread(osPriorityNormal);
    eventThread.start(callback(&TimeQueue, &EventQueue::dispatch_forever));

    // low priority thread for calling Send_transmission()
    Thread TransmitThread(osPriorityNormal);
    TransmitThread.start(callback(&TransmitQueue, &EventQueue::dispatch_forever));

    //call read_sensors 1 every second, automatically defering to the eventThread
    Ticker ReadTicker;
    Ticker TransmitTicker;

    ReadTicker.attach(TimeQueue.event(&Update_time), 3.0f);
    TransmitTicker.attach(TransmitQueue.event(&Send_transmission), 10.0f);

    //This services interrupts. No idea how it works, how often it should be called etc.
    //but it works for now i guess
    Radio::Rx(0);
    for (;;) {
        Radio::service();
    }

    wait(osWaitForever);
}

