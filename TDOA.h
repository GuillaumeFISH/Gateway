#include <mbed.h>


void sort2D(double array[][3], int length);

void swap2rows(double array[][3] , int index, int j);

void buildH(double array[][3], double H[][2], int length);

void buildC(double array[][3], double C[], int length);

void buildD(double H[][2], double D[], double C[], int length);

void buildX(double H[][2], double C[], double D[], double X[][2], int rows);

void inverse2x2(double array[][2]);

double findroots(double X[][2]);

double DM_to_DD(double DM);

void GPS_data();