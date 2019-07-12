#include "radio.h"
#include "UTM.h"
#include "TDOA.h"
#include "MBed_Adafruit_GPS.h"

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
/**********************************************************************/

//Variable to help keep track if a transmission is over
volatile bool txDone;

//Configure GPS objects and NMEA data variable
Serial * gps_Serial = new Serial(D1,D0); //serial object for use w/ GPS
Adafruit_GPS myGPS(gps_Serial); //object of Adafruit's GPS class
char c; //when read via Adafruit_GPS::read(), the class returns single character stored here

//Keep tracks of # of received packets
int received_packets;

//Holds relevant positioning information received from node
double positioning_data[10][3];

//Holds the node #s which received the query
int node_ID[10];

// Defines the EventQueues
EventQueue TransmitQueue;
EventQueue GPSQueue;

//Algorithm to determine position with location information stored in positioning_data[10][3]
void TDOA(){
    //convert lat long in array to decimal degrees format
    //printf("Time: %lf, Lat: %lf, Long: %lf\r\n", positioning_data[0][0], positioning_data[0][1], positioning_data[0][2]);
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

    /*
    printf("Time: %lf, Lat: %5.6f, Long: %5.6f\r\n", positioning_data[0][0], positioning_data[0][1], positioning_data[0][2]);
    printf("Time: %lf, Lat: %5.6f, Long: %5.6f\r\n", positioning_data[1][0], positioning_data[1][1], positioning_data[1][2]);
    printf("Time: %lf, Lat: %5.6f, Long: %5.6f\r\n", positioning_data[2][0], positioning_data[2][1], positioning_data[2][2]);
    */

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
    printf("xsource: %5.6f, ysource: %5.6f\r\n", xsource, ysource);
    GPS_data();

    /* Hardcode gps data for testing.
    if (!myGPS.fix) {
        myGPS.latitude = 5051.95; 
        myGPS.longitude = -10635.49;
        myGPS.lat = 'N';
        myGPS.lon = 'W';
    }
    */

    //TODO: LatLontoUTMXY requires lat and long to be signed appropriately according to heading,
    //need to add similar code as found in rxdonecb to multiply lat/long by -1 when appropriate
    double localLat = DM_to_DD(myGPS.latitude);
    double localLon = DM_to_DD(myGPS.longitude);
    LatLonToUTMXY (localLat, localLon, 0, localLat, localLon);
    printf("localLat: %5.6f, localLon: %5.6f\r\n", localLat, localLon);
    int latdiff = localLat - xsource;
    int londiff = localLon - ysource;
    int error = sqrtf(powf(latdiff, 2) + powf(londiff, 2));
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
    printf("Query sent\r\n");
}

void rxDoneCB(uint8_t size, float rssi, float snr)
{
    printf("------RECEIVING PACKET------\r\n");
    printf("RSSI: %.1fdBm  SNR: %.1fdB\r\n", rssi, snr);

    //Unpacking the transmission
    float sent_time = (float)(Radio::radio.rx_buf[0] << 24 | Radio::radio.rx_buf[1] << 16 | Radio::radio.rx_buf[2] << 8 | Radio::radio.rx_buf[3]);
    unsigned int uint_latitude = (Radio::radio.rx_buf[4] << 24 | Radio::radio.rx_buf[5] << 16 | Radio::radio.rx_buf[6] << 8 | Radio::radio.rx_buf[7]);
    unsigned int uint_longitude = (Radio::radio.rx_buf[8] << 24 | Radio::radio.rx_buf[9] << 16 | Radio::radio.rx_buf[10] << 8 | Radio::radio.rx_buf[11]);
    unsigned int uint_deviceid = Radio::radio.rx_buf[12];
    double dbl_receivedtime = Radio::radio.rx_buf[13] << 16 | Radio::radio.rx_buf[14] << 8 | (Radio::radio.rx_buf[15] & 0xFF);
    int8_t int8_latlon = Radio::radio.rx_buf[16];

    //Converting lat/long/alt to appropriate format
    float fl_latitude = ((float)uint_latitude) / 100;
    float fl_longitude = ((float)uint_longitude) / 100;

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
    printf("Location: %5.6f%c, %5.6f%c\r\n", fl_latitude, c_lat, fl_longitude, c_lon);
    printf("Epoch time: %d\r\n", sent_time);
    printf("Received timestamp: %5.0f\r\n", dbl_receivedtime);
    printf("------PACKET  RECEIVED------\r\n\r\n");

    //Build the received message's row vector
    positioning_data[received_packets][0] = sent_time + dbl_receivedtime/1000000;
    positioning_data[received_packets][1] = fl_latitude;
    positioning_data[received_packets][2] = fl_longitude;

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
    printf("Sending Query\r\n");

    txDone = false;
    Radio::Send(1, 0, 0, 0);
    while (!txDone) {
        Radio::service();
    }
    printf("Done servicing\r\n\r\n");
    Radio::Rx(0);
}

//Collects and parses GPS data
void GPS_data() {
    printf("Got to gps_data\r\n");
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

int main()
{   
    //GPS Settings
    myGPS.sendCommand(PMTK_STANDBY);
    wait(1);
    myGPS.sendCommand(PMTK_AWAKE);
    wait(1);
    myGPS.sendCommand(PMTK_SET_BAUD_57600);
    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
    myGPS.sendCommand(PGCMD_NOANTENNA);

    //Initialize tranceiver settings
    Radio::Init(&rev);
    Radio::Standby();
    Radio::LoRaModemConfig(BW_KHZ, SPREADING_FACTOR, 1);
    Radio::SetChannel(CF_HZ);
    Radio::set_tx_dbm(TX_DBM);
               // preambleLen, fixLen, crcOn, invIQ
    Radio::LoRaPacketConfig(8, false, true, false);
    
    //High priority thread for calls to GPS_data()
    Thread gpsThread(osPriorityHigh);
    gpsThread.start(callback(&GPSQueue, &EventQueue::dispatch_forever));
    GPSQueue.event(&GPS_data);

    //Normal priority thread for calling Send_transmission()
    Thread TransmitThread(osPriorityNormal);
    TransmitThread.start(callback(&TransmitQueue, &EventQueue::dispatch_forever));

    //Ticker object and EventQueue for calling Send_transmission periodically
    Ticker TransmitTicker;
    TransmitTicker.attach(TransmitQueue.event(&Send_transmission), 10.0f);

    Radio::Rx(0);
    for (;;) {
        Radio::service();
    }

    wait(osWaitForever);
}

