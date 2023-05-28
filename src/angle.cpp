//======================================================$
// Name        : angle.cpp
// Author      : Atila
// Version     : 0.1.0
// Copyright   : Your copyright notice
// Description : Monitoreo remoto de la caldera
//======================================================$

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <math.h>

#include "angle.hpp"
#include "capture.hpp"

#define epsilon 8
#define THRE_LINE 7

using namespace cv;
using namespace std;

extern int debug;

int find_angle(Mat& src)
{
    // Declare find circles and lines
    Vec4i circ;		// xc, yc, r, R
    Mat dst, cdst, cdstP;

/*
	if (debug == 1) {
		imwrite("input-capture.jpg", src);
	}
*/
	
    // Creo la imagen gris
    Mat gray;
    cvtColor(src, gray, COLOR_BGR2GRAY);

    medianBlur(gray, gray, 5);
 
 /*   
     if (debug == 1) {
    	cout << "Estoy en angle.cpp" << endl;
    	Mat gray_r;
    	resize(gray, gray_r, Size(gray.cols/2, gray.rows/2), INTER_LINEAR_EXACT);
    	imwrite("gray_pre_hougcirc.jpg", gray_r);
    }
*/

    // Detecto el circulo central.

    vector<Vec3f> circles1;		// Circulo interior
    vector<Vec3f> circles2;		// Circulo exterior

    // Circulo interior
    HoughCircles(gray, circles1, HOUGH_GRADIENT, 1,
                 gray.rows/1.5,  // change this value to detect circles with different distances to each other
                 100, 30, 20, 70 // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
    );
	if (debug == 1) {
		cout << "cantidad de circulos en primera pasada:  " << circles1.size() << endl;
	}
	
    // Circulo exterior
    HoughCircles(gray, circles2, HOUGH_GRADIENT, 1,
                 gray.rows/1.5,  // change this value to detect circles with different distances to each other
                 100, 30, 40, 180 // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
    );
	if (debug == 1) {
		cout << "cantidad de circulos en segunda pasada:  " << circles2.size() << endl;
	}


    // Busco los dos circulos concéntricos
    for (size_t i = 0; i < circles1.size(); i++) {
    	Vec3i c1 = circles1[i];
    	for (size_t j = 0; j < circles2.size(); j++) {
    		Vec3i c2 = circles2[j];
    		if ((abs(c1[0]-c2[0]) <= epsilon) && (abs(c1[1]-c2[1]) <= epsilon)) {
            	circ[0] = (int)round((c1[0] + c2[0]) / 2);
            	circ[1] = (int)round((c1[1] + c2[1]) / 2);
            	circ[2] = c1[2];
            	circ[3] = c2[2];
    		} else {
				if  (debug == 1) {
					cout << "No se encontraron los circulos concentricos" << endl;
				}
    			return -1;
    		}
    	}
    }
    
   	int radius;
    Point center = Point(circ[0], circ[1]);
    // circle center
    circle( src, center, 1, Scalar(0,100,100), 3, LINE_AA);
    // circle outline
    radius = circ[2];
    circle( src, center, radius, Scalar(255,0,255), 3, LINE_AA);
    // circle outline
    radius = circ[3];
    circle( src, center, radius, Scalar(255,0,255), 3, LINE_AA);
  
    
    // Imprimo para ver los resultados
    if (debug == 1) {
		cout << "Centro y radios guardados: " << circ << endl;
	}
	/*
	 * Verfico si el algoritmo para definir la roi es adecuado.
	 * Partiendo del radio exterior encontrado aplico estas relaciones:
	 * La distancia desde el centro del circulo hasta el borde del display
	 * seria d_b = 5/3 * r.
	 * El ancho del display seria: a_d = 3/2 * r.
	 */ 

 
	 int x0_roi, y0_roi, x1_roi, y1_roi;
	 float_t d_b = 3.35;
	 float_t a_d = 2.8;
	 
	 x0_roi = circ[0]-circ[3]*(d_b+a_d);
	 y0_roi = circ[1]-circ[3];
	 x1_roi = circ[0]-circ[3]*d_b;
	 y1_roi = circ[1]+circ[3];
    
     if (debug == 1) {
		cout<<x0_roi<<"; "<<y0_roi<<"; "<<x1_roi<<"; "<<y1_roi<<endl;
	 }
	 
	 Mat ffr_roi;
	 Rect roi_rect;
	 Point p1, p2;
	 
	 p1 = Point(x0_roi, y0_roi);
	 p2 = Point(x1_roi, y1_roi);
	 roi_rect = Rect(p1, p2);
	 ffr_roi = gray(roi_rect);
	 /*
	 if (debug == 1) {
	   imwrite("roi.jpg", ffr_roi);
     }
     */
     find_similarity(roi_rect);

    
    
    // Edge detection   

    Canny(gray, dst, 25, 70, 3, true);

	if (debug == 1) {
		Mat dst_r;
		resize(dst, dst_r, Size(dst.cols/2, dst.rows/2), INTER_LINEAR_EXACT); 
		imwrite("canny.jpg", dst_r);
	}
	
    // Genero los limites entre los cuales tienen que estar los puntos. circ[0] = Xc, circ[1] = Yc, circ[2] = r, circ[3] = R.
    int x1, x2, x3, x4;
    int y1, y2, y3, y4;
    x1 = circ[0] - circ[3] - epsilon;		// Xc - R
    x2 = circ[0] - circ[2] + epsilon;		// Xc - r
    x3 = circ[0] + circ[3] + epsilon;		// Xc + R
    x4 = circ[0] + circ[2] - epsilon;		// Xc + r

    y1 = circ[1] - circ[3] - epsilon;		// Yc - R
    y2 = circ[1] - circ[2] + epsilon;		// Yc - r
    y3 = circ[1] + circ[3] + epsilon;		// Yc + R
    y4 = circ[1] + circ[2] - epsilon;		// Yc + r

	if (debug == 1) {
		cout << "limites x:  " << x1 << ", " << x2 << ", " << x3 << ", " << x4 << endl;
		cout << "limites y:  " << y1 << ", " << y2 << ", " << y3 << ", " << y4 << endl;
		cout << endl << "Estimo el ángulo de la aguja" << endl << endl;
	}
	
    // Probabilistic Line Transform
    vector<Vec4i> linesP; // will hold the results of the detection
    double min_line;
    min_line = (double)(circ[3] - circ[2] - THRE_LINE*epsilon - 20);
    int flag_line;
    flag_line = 0;
    int cuadrante;

    HoughLinesP(dst, linesP, 1, CV_PI/180, 50, min_line, 10 ); // runs the actual detection
    // Draw the lines
    for( size_t i = 0; i < linesP.size(); i++ )
    {
        Vec4i l = linesP[i];

        // Verifico que los vértices en x e y estén dentro de los limites estipulados.
        /*
         * 		---------> +x
         * 		| 1 | 2
         * 		|-------
         * 		| 4 | 3
         * 		v
         * 		+y
         *
         * Siempre los vértices del vector encontrado se devuelven como:
         * x0 menor valor de x, y0 punto asociado (puede ser mayor o menor),
         * x1, y1 punto que indicará el otro vértice del vector. Teniendo en
         * cuenta esto y que el centro de la cruz representa el origen de la
         * aguja se pueden dar los siguientes casos de vector válido en cada
         * cuadrante: x0 < x1 siempre
         * C1- y0 < y1
         * C2- y0 > y1
         * C3- y0 < y1
         * C4- y0 > y1
         *
         * El ángulo se mide desde la horizontal. Siempre será positivo. La
         * indicación de la posición de la aguja la dará el ángulo y el
         * cuadrante donde se encuentre.
         */

         // cuadrantes 1 y 4
        if ((l[0] <= circ[0]) && (l[0] >= x1))  {
        	if ((l[1] <= circ[1]) && (l[1] >= y1)) {
        		if (l[1] <= l[3]) {
            		if (debug == 1) {
            			cout << endl << "cuadrante 1" << endl;
            		}
            		flag_line = 1;
            		cuadrante = 1;
        		}
        	} else if ((l[1] >= circ[1]) && (l[1] <= y3)) {
        		if (l[1] >= l[3]) {
            		if (debug == 1) {
            			cout << endl << "cuadrante 4" << endl;
            		}
            		flag_line = 1;
            		cuadrante = 4;
        		}
        	}
        }
        // cuadrantes 2 y 3
        if ((l[0] >= circ[0]) && (l[0] <= x3)) {
        	if ((l[1] <= circ[1]) && (l[1] >= y1)) {
        		if (l[1] >= l[3]) {
            		if (debug == 1) {
            			cout << endl << "cuadrante 2" << endl;
            		}
            		flag_line = 1;
            		cuadrante = 2;
        		}
        	} else if ((l[1] >= circ[1]) && (l[1] <= y3)) {
        		if (l[1] <= l[3]) {
            		if (debug == 1) {
            			cout << endl << "cuadrante 3" << endl;
            		}
            		flag_line = 1;
            		cuadrante = 3;
        		}
        	}
        }

        // Determino si las lineas pueden o no pertenecer al manómetro. Estimo su longitud.
        if (flag_line == 1) {
			if (debug == 1) {
				cout << "Vector" << linesP[i] << endl;
			}

        	// Cálculo el ángulo que forma la aguja con la horizontal
        	double angle;
        	int dy;
        	int dx;
        	dy = l[3] - l[1];
        	dx = l[2] - l[0];
        	if (debug == 1) {
				cout << "dy: " << dy << " dx: " << dx << endl;
			}

        	angle = atan (double(l[3] - l[1]) / double(l[2] - l[0])) * 180.0 / CV_PI;
        	if (debug == 1) {
				cout << "Ángulo formado: " << angle << endl;
			}

        	int length_line;
        	length_line = (int)round(sqrt(dx*dx+dy*dy));
        	if (debug == 1) {
				cout << "Longitud de la linea: " << length_line << endl;
			}
        	if (abs(length_line - (circ[3]-circ[2])) > (circ[3] - circ[2])*0.5) {
				if (debug == 1) {
					cout << "Probablemente no sea la aguja" << endl << endl;
				}
        	}
        	else {
				if (debug == 1) {
					cout << "-> -> -> Tiene que ser la aguja" << endl;
				}

				if (debug == 1) {
                    cout << circ << ";" << linesP[i] << ";" << ";" << cuadrante << ";" << length_line << ";" << angle << endl;
				}
				angle = abs(angle);
				cout << "S,";
                cout << circ[0] << "," << circ[1] << "," << circ[2] << "," << circ[3] << ",";
                cout << linesP[i][0] << "," << linesP[i][1] << "," << linesP[i][2] << "," << linesP[i][3] << ",";
                cout << cuadrante << "," << length_line << "," << angle;
                cout <<",E" << endl;
        	}

        	line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
        	line( src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);

        	flag_line = 0;
        }
    }

    // Show results
    if (debug == 1) {
		resize(src, src, Size(src.cols/2, src.rows/2), INTER_LINEAR_EXACT);
		imwrite("imagen-final.jpg", src);
	}
    return 0;
}

