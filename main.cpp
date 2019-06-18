#include "radio.h"
#include "UTM.h"
#include "TDOA.h"

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

#define M_PI           3.141592  /* pi */
#define RESPONSE_THRESH 3

//Define serial and led pins
DigitalOut myled(LED1);
Serial pc(PA_11,PA_12);
/**********************************************************************/
volatile bool txDone;

//Keep tracks of # of received packets
int received_packets;

//Holds relevant positioning information received from node
double positioning_data[10][3];

/* Defines the two queues used, one for events and one for printing to the screen */
EventQueue TimeQueue;
EventQueue TransmitQueue;

/* Defines the timer */
Timer t;
time_t whattime;
int64_t usTime1 = 0, usTime2 = 0, usDeltaTime = 0;


void TDOA(){
    //convert lat long in array to decimal degrees format
    //printf("Time: %lf, Lat: %lf, Long: %lf\r\n", positioning_data[0][0], positioning_data[0][1], positioning_data[0][2]);
    for(int i = 0; i < received_packets; i++){
        positioning_data[i][1] = DM_to_DD(positioning_data[i][1]);
        positioning_data[i][2] = DM_to_DD(positioning_data[i][2]);
    }

    //printf("Time: %lf, Lat: %lf, Long: %lf\r\n", positioning_data[0][0], positioning_data[0][1], positioning_data[0][2]);
    //Convert lat long in array to UTM format
    for(int i = 0; i < received_packets; i++){
        LatLonToUTMXY(positioning_data[i][1], positioning_data[i][2], 0, positioning_data[i][1], positioning_data[i][2]);
    }
    //printf("Time: %lf, Lat: %lf, Long: %lf\r\n", positioning_data[0][0], positioning_data[0][1], positioning_data[0][2]);

    //Sort matrix according to time of arrival (column 0)
    sort2D(positioning_data, received_packets);

    //Build H matrix (Shift coords of each node according to reference node)
    double H[received_packets-1][2];
    buildH(positioning_data, H, received_packets);

    //printf("Time: %lf, Lat: %lf, Long: %lf\r\n", positioning_data[0][0], positioning_data[0][1], positioning_data[0][2]);
    //printf("Time: %lf, Lat: %lf, Long: %lf\r\n", positioning_data[1][0], positioning_data[1][1], positioning_data[1][2]);
    //printf("Time: %lf, Lat: %lf, Long: %lf\r\n", positioning_data[2][0], positioning_data[2][1], positioning_data[2][2]);

    //Build C matrix
    double C[received_packets-1];
    buildC(positioning_data, C, received_packets);

    //Build D matrix
    double D[received_packets-1];
    buildD(H, D, C, received_packets-1);

    //Build X matrix
    double X[2][2];
    buildX(H, C, D, X, received_packets-1);

    //Finds the positive root of the polynomial, which is r1
    double root;
    root = findroots(X);

    //Solved for the position of the mobile station and print results
    double xm = root * X[0][0] + X[0][1];
    double ym = root * X[1][0] + X[1][1];
    double xsource = xm + positioning_data[0][1];
    double ysource = ym + positioning_data[0][2];
    printf("xsource: %lf, ysource: %lf\r\n", xsource, ysource);
}

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

    //This is really wonky
    //Unpacking the transmission
    int sent_time = (int)(Radio::radio.rx_buf[0] << 24 | Radio::radio.rx_buf[1] << 16 | Radio::radio.rx_buf[2] << 8 | Radio::radio.rx_buf[3]);
    uint16_t sent_temp1 = Radio::radio.rx_buf[4] << 8 | Radio::radio.rx_buf[5];
   //float fl_temp1 = ((float)sent_temp1)/100;

                               //first 8 bits                 next 8 bits                   Last 4 bits
    unsigned int uint_latitude = Radio::radio.rx_buf[4] << 12 | Radio::radio.rx_buf[5] << 4 | Radio::radio.rx_buf[6] >> 4; 

                                //First 4 bits                            next 8 bits                    next 8 bits                    Last 4 bits
    unsigned int uint_longitude = (Radio::radio.rx_buf[6] & 0xF) << 20 | Radio::radio.rx_buf[7] << 12 | Radio::radio.rx_buf[8] << 4 | Radio::radio.rx_buf[9] >> 4;
    
                                //First 4 bits                            next 8 bits                     next 8 bits                    Last 4 bits
    unsigned int uint_altitude = (Radio::radio.rx_buf[9] & 0xF) << 20 | Radio::radio.rx_buf[10] << 12 | Radio::radio.rx_buf[11] << 4 | Radio::radio.rx_buf[12] >> 4;
   
    unsigned int uint_latlong = Radio::radio.rx_buf[12] & 0xF;
   
    unsigned int uint_deviceid = Radio::radio.rx_buf[13];

    double dbl_receivedtime = Radio::radio.rx_buf[14] << 16 | Radio::radio.rx_buf[14] << 8 | (Radio::radio.rx_buf[14] & 0xFF);
   
    //Converting lat/long/alt to appropriate format
    float fl_latitude = ((float)uint_latitude) / 100;
    float fl_longitude = ((float)uint_longitude) / 100;
    float fl_altitude = ((float)uint_altitude) / 100;

    //Unwrapping latlong packet
    char c_lat;
    char c_lon;
    switch(uint_latlong){
        case 0:
            c_lat = 'N';
            c_lon = 'E';
            break;
        case 1:
            c_lat = 'N';
            c_lon = 'W';
            fl_longitude *= -1;
            break;
        case 2:
            c_lat = 'S';
            c_lon = 'E';
            fl_latitude *= -1;
            break;
        case 3: 
            c_lat = 'S';
            c_lon = 'W';
            fl_longitude *= -1;
            fl_latitude *= -1;
            break;
        //Error case
        default:
            c_lat = 'Z';
            c_lon = 'Z';
    }
    printf("Device id: %d\r\n", uint_deviceid);
    printf("Location: %5.2f%c, %5.2f%c\r\n", fl_latitude, c_lat, fl_longitude, c_lon);
    printf("Altitude: %5.2f\r\n", fl_altitude);
    printf("Epoch time: %d\r\n", sent_time);
    printf("Received timestamp: %5.0f\r\n", dbl_receivedtime);

    printf("------PACKET  RECEIVED------\r\n\r\n");

    positioning_data[received_packets][0] = sent_time/100000000000; //+ dbl_receivedtime/1000000; commented out for testing
    positioning_data[received_packets][1] = fl_latitude;
    positioning_data[received_packets][2] = fl_longitude;

    received_packets++;
    //Start analysis if enough responses have been received
    if(received_packets == RESPONSE_THRESH)
        TDOA();
        
    Radio::Rx(0);
}

void Send_transmission() {
    received_packets = 0;
    //Building the payload
    Radio::radio.tx_buf[0] = 0xAB;
    pc.printf("Sending Query\r\n");

    txDone = false;
    Radio::Send(1, 0, 0, 0);   /* begin transmission */
    while (!txDone) {
        Radio::service();
    }
    printf("Done servicing\r\n\r\n");
    Radio::Rx(0);
}

//Many of these (likely all) are interrupts. Associate to them the name of a function you want executed
//when the interrupt is triggered. Not all of them are associated to a pin.
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
    printf("\r\nReset Gateway...\r\n");
    
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
    
    /*MATLAB port testing
    //double array[4][3] = {{3,4,5},{1,2,1},{5,1,4},{4,5,8}};
    //double array[5][3] = {{3,4,5},{1,2,1},{5,1,4},{4,5,8},{1.4,8,2}};
    //double array[5][3] = {{0.00004, 317090.99, 5874020.42},{0.00003193333, 323242.50, 5859811.98},
    //{0.00003986666, 339229.73, 5871615.81},{0.00003766666,338950,5867016},{0.00003186666,318484,5865669}};
    double array[5][3] = {{0.00001647806,392968,5635967},{0.0000211146,389057,5629856},{0.00003699225,376895,5636518}
    ,{0.00002124803,390609,5641925},{0.00002054754,383773,5640583}};
    int rows = 5;
    sort2D(array, rows);
    printf("\r\nReceived data Matrix sorted\r\n");
    printf("%lf %lf %lf\r\n", array[0][0],array[0][1],array[0][2]);
    printf("%lf %lf %lf\r\n", array[1][0],array[1][1],array[1][2]);
    printf("%lf %lf %lf\r\n", array[2][0],array[2][1],array[2][2]);
    printf("%lf %lf %lf\r\n", array[3][0],array[3][1],array[3][2]);
    printf("%lf %lf %lf\r\n", array[4][0],array[4][1],array[4][2]);

    
    double H[rows-1][2];
    buildH(array, H, rows);
    printf("\r\nH Matrix \r\n");
    printf("%lf %lf \r\n", H[0][0], H[0][1]);
    printf("%lf %lf \r\n", H[1][0], H[1][1]);
    printf("%lf %lf \r\n", H[2][0], H[2][1]);
    printf("%lf %lf \r\n", H[3][0], H[3][1]);

    double C[rows-1];
    buildC(array, C, rows);
    printf("\r\nC Matrix \r\n");
    printf("%lf \r\n", C[0]);
    printf("%lf \r\n", C[1]);
    printf("%lf \r\n", C[2]);
    printf("%lf \r\n", C[3]);
    

    double D[rows-1];
    buildD(H, D, C, rows-1);
    printf("\r\nD Matrix \r\n");
    printf("%lf \r\n", D[0]);
    printf("%lf \r\n", D[1]);
    printf("%lf \r\n", D[2]);
    printf("%lf \r\n", D[3]);

    double X[2][2];
    buildX(H, C, D, X, rows-1);

    double root;
    root = findroots(X);
    printf("Root: %lf\r\n", root);

    double xm = root * X[0][0] + X[0][1];
    double ym = root * X[1][0] + X[1][1];
    double xsource = xm + array[0][1];
    double ysource = ym + array[0][2];
    printf("xsource: %lf, ysource: %lf\r\n", xsource, ysource);
    */

    /*Utm Conversion Testing
    double x;
    double y;
    double lon = -114.112369;
    double lat = 51.098694;
    int zone = 0;
    zone = LatLonToUTMXY(lat, lon, zone, x, y);
    printf("Zone: %d x: %lf y: %lf\r\n",zone, x, y);

    zone = 11;
    UTMXYToLatLon (x, y, zone, false, lat, lon);
    printf("lat: %lf lon: %lf\r\n",lat, lon);
    */

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
    TransmitTicker.attach(TransmitQueue.event(&Send_transmission), 20.0f);

    //This services interrupts. No idea how it works, how often it should be called etc.
    //but it works for now i guess
    Radio::Rx(0);
    for (;;) {
        Radio::service();
    }

    wait(osWaitForever);
}

