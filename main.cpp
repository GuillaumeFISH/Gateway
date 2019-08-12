#include "radio.h"
#include "UTM.h"
#include "TDOA.h"
#include "MBed_Adafruit_GPS.h"

#if defined(SX127x_H)
    #define BW_KHZ              125
    #define SPREADING_FACTOR    10
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
#define RESPONSE_THRESH 4
/**********************************************************************/

volatile bool txDone;                    //Keeps track if a transmission is over

Serial * gps_Serial = new Serial(D1,D0); //serial object for use w/ GPS
Adafruit_GPS myGPS(gps_Serial);          //object of Adafruit's GPS class
char c;                                  //when read via Adafruit_GPS::read(), the class returns single character stored here

int received_packets;                   //Keep tracks of # of received packets

double positioning_data[10][3];         //Holds relevant positioning information received from node
int node_ID[10];                        //Holds the node #s which received the query

EventQueue TransmitQueue;               // Defines the EventQueues
EventQueue GPSQueue;

/***********************************************************************/

//Algorithm to determine position with location information stored in positioning_data[10][3]
void TDOA(){
    //convert lat long in array to decimal degrees format
    for(int i = 0; i < received_packets; i++){
        positioning_data[i][1] = DM_to_DD(positioning_data[i][1]);
        positioning_data[i][2] = DM_to_DD(positioning_data[i][2]);
    }

    //Convert lat long in array to UTM format
    for(int i = 0; i < received_packets; i++){
        LatLonToUTMXY(positioning_data[i][1], positioning_data[i][2], 0, positioning_data[i][1], positioning_data[i][2]);
    }

    //Sort matrix according to time of arrival (column 0)
    sort2D(positioning_data, received_packets);

    //Build H matrix (Shift coords of each node according to reference node)
    double H[received_packets-1][2];
    buildH(positioning_data, H, received_packets);

    //printf("Time: %10.15f, Lat: %5.6f, Long: %5.6f\r\n", positioning_data[0][0], positioning_data[0][1], positioning_data[0][2]);
    //printf("Time: %10.15f, Lat: %5.6f, Long: %5.6f\r\n", positioning_data[1][0], positioning_data[1][1], positioning_data[1][2]);
    //printf("Time: %10.15f, Lat: %5.6f, Long: %5.6f\r\n", positioning_data[2][0], positioning_data[2][1], positioning_data[2][2]);
    //printf("Time: %10.15f, Lat: %5.6f, Long: %5.6f\r\n", positioning_data[3][0], positioning_data[3][1], positioning_data[3][2]);
    //printf("Time: %10.15f, Lat: %5.6f, Long: %5.6f\r\n", positioning_data[4][0], positioning_data[4][1], positioning_data[4][2]);
    //printf("Time: %10.15f, Lat: %5.6f, Long: %5.6f\r\n", positioning_data[5][0], positioning_data[5][1], positioning_data[5][2]);
    //printf("Time: %10.15f, Lat: %5.6f, Long: %5.6f\r\n", positioning_data[6][0], positioning_data[6][1], positioning_data[6][2]);

    //Build C matrix
    double C[received_packets-1];
    buildC(positioning_data, C, received_packets);

    //Build D matrix
    double D[received_packets-1];
    buildD(H, D, C, received_packets-1);

    //Build X matrix
    double X[2][2];

    //Build X matrix and find roots of r1
    double root = buildX(H, C, D, X, received_packets-1);

    //Finds the positive root of the polynomial, which is r1. Removed to be able to have 6 nodes
    //double root;
    //root = findroots(X);

    //Solve for the position of the mobile station and print results
    double xm = root * X[0][0] + X[0][1];
    double ym = root * X[1][0] + X[1][1];
    double xsource = xm + positioning_data[0][1];
    double ysource = ym + positioning_data[0][2];
    //GPS_data(); Commented out for indoor testing

    
    myGPS.latitude = 5104.6753;         //Hardcoded coords for indoor testing
    myGPS.longitude = 11408.2705;
    myGPS.lat = 'N';
    myGPS.lon = 'W';
    
    //Make lat/lon negative according to hemisphere. Necessary for UTM conversion
    myGPS.longitude *= -1;
    if(myGPS.lat == 'S')
        myGPS.latitude *= -1;
    if(myGPS.lon == 'W')
        myGPS.longitude *= -1;

    double localLat = DM_to_DD(myGPS.latitude);     //Lat/long of mobile station
    double localLon = DM_to_DD(myGPS.longitude);    //Useful for on-site comparison
    LatLonToUTMXY (localLat, localLon, 0, localLat, localLon);
    int latdiff = localLat - xsource;
    int londiff = localLon - ysource;
    int error = sqrtf(powf(latdiff, 2) + powf(londiff, 2));

    printf("xsource: %5.6f, ysource: %5.6f\r\n", xsource, ysource);
    printf("localLat: %5.6f, localLon: %5.6f\r\n", localLat, localLon);
    printf("Error in reported distance: %dm\r\n", error);
    printf("Number of nodes used in TDOA calc: %d\r\n", received_packets);
    printf("Nodes used in TDOA: ");
    //Print out node IDs' that received query
    for( int i = 0; i < received_packets; i++){
        printf("%d ", node_ID[i]);
    }
    printf("\r\n\r\n");
}

void txDoneCB()
{
    txDone = true;
    //printf("Query sent\r\n");
}

void rxDoneCB(uint8_t size, float rssi, float snr)
{
    printf("------RECEIVING PACKET------\r\n");
    printf("RSSI: %.1fdBm  SNR: %.1fdB\r\n", rssi, snr);

    //Unpacking the transmission
    unsigned int sent_time = (Radio::radio.rx_buf[0] << 24 | Radio::radio.rx_buf[1] << 16 | Radio::radio.rx_buf[2] << 8 | Radio::radio.rx_buf[3]);
    unsigned int uint_latitude = (Radio::radio.rx_buf[4] << 24 | Radio::radio.rx_buf[5] << 16 | Radio::radio.rx_buf[6] << 8 | Radio::radio.rx_buf[7]);
    unsigned int uint_longitude = (Radio::radio.rx_buf[8] << 24 | Radio::radio.rx_buf[9] << 16 | Radio::radio.rx_buf[10] << 8 | Radio::radio.rx_buf[11]);
    unsigned int uint_deviceid = Radio::radio.rx_buf[12];
    //Cycles Test
    //double dbl_receivedtime = Radio::radio.rx_buf[13] << 16 | Radio::radio.rx_buf[14] << 8 | (Radio::radio.rx_buf[15] & 0xFF);
    uint32_t CyclesElapsed = (Radio::radio.rx_buf[13] << 24 | Radio::radio.rx_buf[14] << 16 | Radio::radio.rx_buf[15] << 8 | Radio::radio.rx_buf[16]);

    int8_t int8_latlon = Radio::radio.rx_buf[16];

    //Converting lat/long/alt to appropriate format
    double fl_latitude = ((double)uint_latitude) / 100;
    double fl_longitude = ((double)uint_longitude) / 100;

    //Unwrapping latlong packet
    char c_lat;
    char c_lon;
    switch(int8_latlon){
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
    printf("Location: %5.6f, %5.6f\r\n", DM_to_DD(fl_latitude), DM_to_DD(fl_longitude));
    printf("Epoch time: %d\r\n", sent_time);
    printf("Cycles Elapsed %u\r\n", CyclesElapsed);
    printf("------PACKET  RECEIVED------\r\n\r\n");

    //Build the received message's row vector
    double sent_timefl = (double)sent_time;
    double decimal = CyclesElapsed * (1 / (double)SystemCoreClock);
    positioning_data[received_packets][0] = 100000+/*sent_timefl +*/ decimal; //big numbers were causing a problem maybe
    positioning_data[received_packets][1] = fl_latitude;
    positioning_data[received_packets][2] = -1*fl_longitude; //Maybe need to multiply by -1

    //Log the node's ID
    node_ID[received_packets] = uint_deviceid;

    received_packets++;
        
    Radio::Rx(0);
}

void Send_transmission() {
    //# of responses check is here since Send_transmission() is called periodically
    //and thus gateway only executes TDOA after certain period and if it received enough responses.
    if(received_packets >= RESPONSE_THRESH)
        TDOA();

    received_packets = 0;

    //Building the payload
    Radio::radio.tx_buf[0] = 0xAB;
    //printf("Sending Query\r\n");

    txDone = false;
    Radio::Send(1, 0, 0, 0);
    while (!txDone) {
        Radio::service();
    }
    //printf("Done servicing\r\n\r\n");
    Radio::Rx(0);
}

//Collects and parses GPS data
void GPS_data() {
    do{
        c = myGPS.read();   //queries the GPS
        //if (c) { pc.printf("%c", c); } //this line will echo the GPS data if not paused
        //check if we recieved a new message from GPS, if so, attempt to parse it,
        if (myGPS.newNMEAreceived() == true)
        {
            if (myGPS.parse(myGPS.lastNMEA()) == true)
            {
                break;
            }
        }
    } while(myGPS.newNMEAreceived() == false);
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

//Maybe not needed
void Request_Correction_Update(){
    Radio::radio.tx_buf[0] = 0xAC;

    txDone = false;
    Radio::Send(1, 0, 0, 0);
    while (!txDone) {
        Radio::service();
    }
    Radio::Rx(0);
}
InterruptIn PPS(PC_2, PullNone);
int main()
{   
    //GPS Settings
    myGPS.begin(57600);
    wait(1);
    myGPS.sendCommand(PMTK_AWAKE);
    wait(1);
    myGPS.sendCommand(PMTK_SET_BAUD_57600);
    wait(1);
    myGPS.sendCommand(PMTK_STANDBY);
    wait(1);
    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
    myGPS.sendCommand(PGCMD_NOANTENNA);

    //Initialize tranceiver settings
    Radio::Init(&rev);
    Radio::Standby();
    Radio::LoRaModemConfig(BW_KHZ, SPREADING_FACTOR, 1);
    Radio::SetChannel(CF_HZ);
    Radio::set_tx_dbm(TX_DBM);
               // preambleLen, fixLen, crcOn, invIQ
    Radio::LoRaPacketConfig(8, false, true, false);

    SystemCoreClockUpdate();
    //Request_Correction_Update();
    //Outdoor test simulation

    /*
    positioning_data[0][0] = 6.40242924e-7;  //site 1
    positioning_data[0][1] = 5104.72998;
    positioning_data[0][2] = -11408.12988;

    positioning_data[1][0] = 9.69737538e-7;  //site 2
    positioning_data[1][1] = 5104.689941;
    positioning_data[1][2] = -11408.4502;

    positioning_data[2][0] = 7.26169035e-7;  //site 3
    positioning_data[2][1] = 5104.560059;
    positioning_data[2][2] = -11408.44043;


    positioning_data[3][0] = 0.00000170117;  //site 4
    positioning_data[3][1] = 5104.6826;
    positioning_data[3][2] = -11407.8268;

    positioning_data[4][0] = 8.00553828e-7;  //site 5ish
    positioning_data[4][1] = 5104.569824;
    positioning_data[4][2] = -11408.16016;

    //swapped with site 4
    positioning_data[5][0] = 0.00000111557;  //site 6
    positioning_data[5][1] = 5104.850098;
    positioning_data[5][2] = -11408.29981;

    positioning_data[6][0] = 0.00000167449;  //site 7
    positioning_data[6][1] = 5104.5974;
    positioning_data[6][2] = -11408.6857;

    positioning_data[7][0] = 0.00000129663;  //south of vehicle
    positioning_data[7][1] = 5104.4724;
    positioning_data[7][2] = -11408.1853;
    

    positioning_data[0][0] = 0.00000121083;  //site 10
    positioning_data[0][1] = 5104.8501;
    positioning_data[0][2] = -11408.3799;

    positioning_data[1][0] = 0.00000137761;  //site 11
    positioning_data[1][1] = 5104.7402;
    positioning_data[1][2] = -11408.9199;

    positioning_data[2][0] = 0.00000137761;  //site 12
    positioning_data[2][1] = 5104.48;
    positioning_data[2][2] = -11408.3799;

    //positioning_data[3][0] = 0.00000126754;  //site 13
    //positioning_data[3][1] = 5104.52;
    //positioning_data[3][2] = -11408.0401;

    positioning_data[3][0] = 0.000000590408449;  //site 14
    positioning_data[3][1] = 5104.5801;
    positioning_data[3][2] = -11408.21;

    received_packets = 4;
    TDOA();
    */

    //Normal priority thread for calling Send_transmission()
    Thread TransmitThread(osPriorityHigh);
    TransmitThread.start(callback(&TransmitQueue, &EventQueue::dispatch_forever));
    
    //Ticker object and EventQueue for calling Send_transmission periodically
    Ticker TransmitTicker;
    TransmitTicker.attach(TransmitQueue.event(&Send_transmission), 5.0f);
    //PPS.fall(TransmitQueue.event(&Send_transmission));
    Radio::Rx(0);
    for (;;) {
        Radio::service();
    }

    wait(osWaitForever);
}

