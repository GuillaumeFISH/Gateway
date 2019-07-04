#include "TDOA.h"
#define V_LIGHT        299792458
// sort2D
// Sorts a 2D array based on the values of the first column
// 
// Inputs:
//     array = N x 3 matrix to be sorted
//     length = # of rows in array (received_packets)
// 
// Returns:
//     Nothing
void sort2D(double array[][3], int length){
                //i<number of rows
    for(int i=1; i<length; i++){
        int index = i;
        for(int j=i-1; j>=0; j--, index--)
            if(array[j][0]>array[index][0])
                swap2rows(array, index,j);
            else
                break;
    }
}
// swap2rows
// Swaps two rows of matrix[index] and matrix[j]
// 
// Inputs:
//     array = N x 3 matrix which has rows to be swapped
//     index = row # to be swapped
//     j = row # to be swapped
// 
// Returns:
//     Nothing
void swap2rows(double array[][3] , int index, int j){
    double temp;
                //i<number of columns
    for(int i=0; i<3; i++){
        temp = array[j][i];
        array[j][i] = array[index][i];
        array[index][i] = temp;
    }
}
// buildH
// Constructs the H matrix, which is filled with the x,y coords of each node,
// shifted by reference node.
// 
// Inputs:
//     array = N x 3 matrix containing node coordinates
//     H = N x 2 matrix where shifted coords will be placed
//     length = # of rows in array (received_packets)
// 
// Returns:
//     Nothing
void buildH(double array[][3], double H[][2], int length){
    for(int i = 0; i<length-1; i++){
        H[i][0] = array[i+1][1] - array[0][1]; //Xn - X0
        H[i][1] = array[i+1][2] - array[0][2]; //Yn - Y0
    }
}

// buildC
// Constructs the C matrix, which contains the range differences
// of all nodes with respect to the reference.
// 
// Inputs:
//     array = N x 3 matrix containing node coordinates
//     C = N x 1 vector where range differences will be placed
//     length = # of rows in array (received_packets)
// 
// Returns:
//     Nothing
void buildC(double array[][3], double C[], int length){
    for(int i = 0; i<length-1; i++){
        C[i] = -1 * (array[i+1][0] - array[0][0]) * V_LIGHT;
    }
}

// buildD
// Constructs the D matrix, which contains 0.5 * ( (x^2 + y^2) - r^2)
// for all nodes.
// 
// Inputs:
//     H = N x 2 H matrix containing node coordinates
//     D = N x 1 vector where results are placed
//     C = N x 1 vector containing node range differences
//     length = # of rows in H  (received_packets-1)
// 
// Returns:
//     Nothing
void buildD(double H[][2], double D[], double C[], int length){
    for(int i = 0; i<length; i++){
        D[i] = 0.5 * ( powf(H[i][0], 2) + powf(H[i][1], 2) - powf(C[i], 2) );
    }
}

// inverse2x2
// Calculates the inverse of a 2x2 matrix
// 
// Inputs:
//     array = 2x2 matrix to be inversed
// 
// Returns:
//     Nothing
void inverse2x2(double array[][2]){
    double invdet = 1 / ( array[0][0] * array[1][1] - array[0][1] * array[1][0] );
    double a = array[0][0];
    array[0][0] = array[1][1] * invdet;
    array[0][1] = array[0][1] * invdet * -1;
    array[1][0] = array[1][0] * invdet * -1;
    array[1][1] = a * invdet;
}

// buildX
// Constructs the X matrix, which contains {xa, xb; ya, yb} for
// equation xm = xa*r1 + xb and ym = ya*r1 + yb which is the location
// of the mobile station
// 
// Inputs:
//     H = N x 2 H matrix containing node coordinates
//     C = N x 1 vector containing node range differences
//     D = N x 1 vector containing 0.5 * ( (x^2 + y^2) - r^2) for all nodes.
//     X = N x 2 vector where xa, xb, ya, yb will be placed
//     rows = # of rows in H (received_packets-1)
// 
// Returns:
//     Nothing
void buildX(double H[][2], double C[], double D[], double X[][2], int rows){
    double Htranspose[2][rows];

    //Transpose of H
    int i, j; 
    for (i = 0; i < 2; i++) 
        for (j = 0; j < rows; j++) 
            Htranspose[i][j] = H[j][i]; 
    /*
    printf("\r\nHTranspose \r\n");
    printf("%lf %lf %lf %lf\r\n", Htranspose[0][0], Htranspose[0][1], Htranspose[0][2], Htranspose[0][3]);
    printf("%lf %lf %lf %lf\r\n", Htranspose[1][0], Htranspose[1][1], Htranspose[1][2], Htranspose[1][3]);
    */
    //Htranspose * H
    int left_col = rows;
    int right_row = 2;
    int k;
    double result[2][2] = {{0,0}, {0,0}};
    for(i=0;i<left_col;i++)
    {
        for(j=0;j<right_row;j++)
        {
            for (k=0;k<left_col;k++)
            {
            result[i][j] = result[i][j] + (Htranspose[i][k] * H[k][j]);
            }
        }
    }
    /*
    printf("\r\nresult\r\n");
    printf("%lf %lf\r\n", result[0][0], result[0][1]);
    printf("%lf %lf\r\n", result[1][0], result[1][1]);
    */
    
    //Inv(H^t*H)
    inverse2x2(result);
    /*
    printf("\r\nInverse Result\r\n");
    printf("%lf %lf\r\n", result[0][0], result[0][1]);
    printf("%lf %lf\r\n", result[1][0], result[1][1]);
    */

    //Inv(H^t*H) * H^t
    double intermediate2[2][rows];
    //initialize with zeros
    for(i=0; i<2; i++){
        for(j=0; j<rows; j++){
            intermediate2[i][j] = 0;
        }
    }
    left_col = 2;
    int right_col = rows; 
    for(i=0;i<left_col;i++)
    {
        for(j=0;j<right_col;j++)
        {
            for (k=0;k<left_col;k++)
            {
            intermediate2[i][j] += (result[i][k] * Htranspose[k][j]);
            }
        }
    }
    /*
    printf("\r\nIntermediate2\r\n");
    printf("%lf %lf %lf %lf\r\n", intermediate2[0][0], intermediate2[0][1], intermediate2[0][2], intermediate2[0][3]);
    printf("%lf %lf %lf %lf\r\n", intermediate2[1][0], intermediate2[1][1], intermediate2[1][2], intermediate2[1][3]);
    */

    //intermediate2 * C
    double x1[2] = {0,0};
    int left_row = 2;
    left_col = rows; //left col = right rows
    for(i=0;i<left_row;i++)
    {
        for(j=0;j<left_col;j++)
        {
            x1[i] += intermediate2[i][j] * C[j];

        }
    }
    /*
    printf("\r\nx1\r\n");
    printf("%lf\r\n", x1[0]);
    printf("%lf\r\n", x1[1]);
    */

    //intermediate2 * D
    double x2[2] = {0,0};
    left_row = 2;
    left_col = rows; //right rows
    for(i=0;i<left_row;i++)
    {
        for(j=0;j<left_col;j++)
        {
            x2[i] += intermediate2[i][j] * D[j];

        }
    }
    /*
    printf("\r\nx2\r\n");
    printf("%lf\r\n", x2[0]);
    printf("%lf\r\n", x2[1]);
    */

    //Finish building X
    X[0][0] = x1[0];
    X[0][1] = x2[0];
    X[1][0] = x1[1];
    X[1][1] = x2[1];
    /*
    printf("\r\nX\r\n");
    printf("%lf %lf\r\n", X[0][0], X[0][1]);
    printf("%lf %lf\r\n", X[1][0], X[1][1]);
    */
}

// findroots
// Constructs a polynomial in the form ax^2 + bx + c, where x is r1,
// and solves for the positive root.
// 
// Inputs:
//     X = N x 2 vector where xa, xb, ya, yb are held, these values
//     are used to build the polynomial.
// 
// Returns:
//     Positive root of polynomial
double findroots(double X[][2]){
    double P[3];
    P[0] = powf(X[0][0],2) + powf(X[1][0],2) - 1;
    P[1] = X[0][0] * X[0][1] * 2 + X[1][0] * X[1][1] * 2;
    P[2] = powf(X[0][1], 2) + powf(X[1][1], 2);

    /*
    printf("\r\nP\r\n");
    printf("%lf %lf %lf\r\n", P[0], P[1], P[2]);
    */
    
    double root1, root2, det;
    det = powf(P[1],2) - 4 * P[0] * P[2]; 
    root1 = ( -1 * P[1] + sqrtf(det) ) / ( 2 * P[0] );
    root2 = ( -1 * P[1] - sqrtf(det) ) / ( 2 * P[0] );
    if(root1 > 0)
        return root1;
    else
        return root2;
}

// DM_to_DD
// Converts the input from decimal minutes format to decimal degrees
// 
// Inputs:
//     DM = Geodetic lat/long in decimal minutes format ie. 30deg 30.18min
// 
// Returns:
//     Decimal degree representation of DM, ie. 30.503 deg
double DM_to_DD(double DM){
    double minutes, degrees;

    DM /= 100;
    minutes = std::modf(DM, &degrees);
    minutes *= 100;

    return (degrees + minutes/60);
}
