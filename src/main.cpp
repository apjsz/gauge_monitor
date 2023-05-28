//============================================================================
// Name        : main.cpp
// Author      : Atila
// Version     : 0.1.0
// Copyright   : Your copyright notice
// Description : Monitoreo remoto de la caldera
//============================================================================

#include <iostream>
#include <string>
#include <string.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "capture.hpp"
#include "angle.hpp"

using namespace cv;
using namespace std;

int debug;


int main(int argc, char* argv[]) {

    double alpha = 1.0; /*< Simple contrast control */
    int beta = 40;       /*< Simple brightness control */

    if (argc < 2) {
       	cout << "Para correrlo en modo debug ejecute el programa con el argumento -d" << endl;
       	cout << "Para correrlo en modo producción ejecute el programa con el argumento -p" << endl;
       	cout << "Secuencia de datos que devuelve en modo -p:" << endl;
       	cout << "centro_x, centro_y, radio_1, radio_2, x_1, y_1, x_2, y_2, longitud_aguja, cuadrante, ángulo" << endl;
       	cout << "centro_x, centro_y, radio_1, radio_2 ubican en la imagen el dial del manómetro" << endl;
       	cout << "(x_1, y_1) y (x_2, y2) posición incial y final de la aguja" << endl;
       	return -1;
    }

  debug = 0;

    if (strcmp(argv[1], "-d") == 0) {
        debug = 1;
    }

    if (debug == 1 ) {
    	cout << "Modo debug" << endl;
    }
    
    Mat src, dst, dst_r, dst_n, dst_nr;
    capture(src);
            
    GaussianBlur(src, dst, Size(0,0), 3);
    addWeighted(src, 1.5, dst, -0.5, 0, dst);

/*
    if (debug == 1) {
    	resize(dst, dst_r, Size(dst.cols/2, dst.rows/2), INTER_LINEAR_EXACT);
    	imwrite("Sharpened reducido.jpg", dst_r);
	}
*/

    dst.convertTo(dst_n, -1, alpha, beta);

/*
    if (debug == 1) {
    	resize(dst_n, dst_nr, Size(dst_n.cols/2, dst_n.rows/2), INTER_LINEAR_EXACT);
    	imwrite("Mejorado.jpg", dst_n);
    }
*/

    find_angle(dst_n);

  return 0;
}
