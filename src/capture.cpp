//======================================================$
// Name        : capture.cpp
// Author      : Atila
// Version     : 0.1.0
// Copyright   : Your copyright notice
// Description : Monitoreo remoto de la caldera
//======================================================$

#include <iostream>
#include <math.h>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <cstdlib>

#include "angle.hpp"

#define VIDEO_LENGTH 10
extern int debug;

using namespace std;
using namespace cv;

double getPSNR(const Mat& I1, const Mat& I2);
Scalar getMSSIM( const Mat& i1, const Mat& i2);

int capture(Mat& frame) {
  VideoCapture cap(0);

  // Set the properties of the capture object
  // Seteado para una raspberry pi camera module v1
  cap.set(CAP_PROP_FRAME_WIDTH, 2592); 		// Máximo ancho en pixeles
  cap.set(CAP_PROP_FRAME_HEIGHT, 1944);		// Máximo alto en pixeles
  cap.set(CAP_PROP_FPS, 4); 			// Cuadros por segundo

  if (!cap.isOpened()) {
    // TODO Debería enviar la información de que no pudo abrir la cámara
    cout << "Fallo la apertura de la cámara" << endl;
    return -1;  	// TODO Falta la definición de los códigos de error. 
  }

  struct timeval start_time, end_time;
  gettimeofday(&start_time, NULL);
  gettimeofday(&end_time, NULL);

  while ((end_time.tv_sec - start_time.tv_sec) < 2) {
	gettimeofday(&end_time, NULL);
	if (!cap.read(frame)) {
	  if (debug == 1) {
	    cout << "error, blank frame grabbed" << endl;
	  }
	  break;
	}
  }

  // Mat frame, gray, edges;
  cap >> frame;
  if (frame.empty()) {
    // TODO Enviar la falla
    cout << "Fallo la captura" << endl;
    return -1;		// TODO Códigos de error
  }

  cap.release();
  
  return 0;
}

int find_similarity(Rect roi) {
  Mat frame, f_g, dst, dst_n;
  double alpha = 1.0;    // Simple contrast control
  int beta = 40;		// Simple brightness control

  VideoCapture cap(0);

  // Set the properties of the capture object
  cap.set(CAP_PROP_FRAME_WIDTH, 2592); 		// Máximo ancho en pixeles
  cap.set(CAP_PROP_FRAME_HEIGHT, 1944);		// Máximo alto en pixeles
  cap.set(CAP_PROP_FPS, 4); 			// Cuadros por segundo

  if (!cap.isOpened()) {
    // TODO Debería enviar la información de que no pudo abrir la cámara
    cout << "Fallo la apertura de la cámara" << endl;
    return -1;  	// TODO Falta la definición de los códigos de error. 
  }

  // Mat frame, gray, edges;
  cap >> frame;
  if (frame.empty()) {
    // TODO Enviar la falla
    cout << "Fallo la captura" << endl;
    return -1;		// TODO Códigos de error
  }
  cvtColor(frame, f_g, COLOR_BGR2GRAY);
  GaussianBlur(f_g, dst, Size(0,0), 3);
  addWeighted(f_g, 1.5, dst, -0.5, 0, dst);
  dst.convertTo(dst_n, -1, alpha, beta);

  Mat dst_roi;
  dst_roi = dst_n(roi);
  
  if (debug == 1) {
	  imwrite("primer-roi.jpg", dst_roi);
  }
  
  struct timeval start_time, end_time;
  gettimeofday(&start_time, NULL);
  gettimeofday(&end_time, NULL);
  
  int flag = 1;
  double psnr, psnr0=0, psnr1=0, psnr2=0, psnr3=0;
  Scalar mssin;
  int counter = 0;
  int blinking = 1;
  
  while ((end_time.tv_sec - start_time.tv_sec) < VIDEO_LENGTH) {
	gettimeofday(&end_time, NULL);
	if (!cap.read(frame)) {
	  if (debug == 1) {
	    cout << "error, blank frame grabbed" << endl;
	  }
	  break;
	}
    cvtColor(frame, f_g, COLOR_BGR2GRAY);
    GaussianBlur(f_g, dst, Size(0,0), 3);
    addWeighted(f_g, 1.5, dst, -0.5, 0, dst);
    dst.convertTo(dst_n, -1, alpha, beta);	
	
	Mat f_roi;
	f_roi = dst_n(roi);
	if (flag == 1) {
		imwrite("roi-loop.jpg", f_roi);
		flag = 0;
	}
	
	psnr = getPSNR(f_roi, dst_roi);
	//mssim = getMSSIM(f_roi, dst_roi);
	
	psnr3 = psnr2;
	psnr2 = psnr1;
	psnr1 = psnr0;
	psnr0 = psnr;

	double delta_psnr;
	delta_psnr = abs(psnr0 - psnr3);

	if (counter > 3) {	
		if (delta_psnr > 1) {
			if (blinking == 1) {
				if (system("sendimage.sh") >= 0) {
					blinking = 0;
				} else {
					cout << "Error llamando script" << endl;
				}
			}
			if (debug == 1) {
				cout << ">>>>>>>> titila" << endl;
			}
		}
	}


	
	if (debug == 1) {
	  cout << "delta psnr:  " << delta_psnr << endl;
	  cout << "PSNR:  " << psnr << endl;
	  //cout << "  MSSIM:  " << mssim << endl;
    }
  }

  cap.release();
  return 0;
}


double getPSNR(const Mat& I1, const Mat& I2)
{
    Mat s1;
    absdiff(I1, I2, s1);       // |I1 - I2|
    //imshow("Diferencias", s1);
    s1.convertTo(s1, CV_32F);  // cannot make a square on 8 bits
    s1 = s1.mul(s1);           // |I1 - I2|^2
    Scalar s = sum(s1);        // sum elements per channel
    double sse = s.val[0] + s.val[1] + s.val[2]; // sum channels
    if( sse <= 1e-10) // for small values return zero
        return 0;
    else
    {
        double mse  = sse / (double)(I1.channels() * I1.total());
        double psnr = 10.0 * log10((255 * 255) / mse);
        return psnr;
    }
}

Scalar getMSSIM( const Mat& i1, const Mat& i2)
{
    const double C1 = 6.5025, C2 = 58.5225;
    /***************************** INITS **********************************/
    int d = CV_32F;
    Mat I1, I2;
    i1.convertTo(I1, d);            // cannot calculate on one byte large values
    i2.convertTo(I2, d);
    Mat I2_2   = I2.mul(I2);        // I2^2
    Mat I1_2   = I1.mul(I1);        // I1^2
    Mat I1_I2  = I1.mul(I2);        // I1 * I2
    /*************************** END INITS **********************************/
    Mat mu1, mu2;                   // PRELIMINARY COMPUTING
    GaussianBlur(I1, mu1, Size(11, 11), 1.5);
    GaussianBlur(I2, mu2, Size(11, 11), 1.5);
    Mat mu1_2   =   mu1.mul(mu1);
    Mat mu2_2   =   mu2.mul(mu2);
    Mat mu1_mu2 =   mu1.mul(mu2);
    Mat sigma1_2, sigma2_2, sigma12;
    GaussianBlur(I1_2, sigma1_2, Size(11, 11), 1.5);
    sigma1_2 -= mu1_2;
    GaussianBlur(I2_2, sigma2_2, Size(11, 11), 1.5);
    sigma2_2 -= mu2_2;
    GaussianBlur(I1_I2, sigma12, Size(11, 11), 1.5);
    sigma12 -= mu1_mu2;
    Mat t1, t2, t3;
    t1 = 2 * mu1_mu2 + C1;
    t2 = 2 * sigma12 + C2;
    t3 = t1.mul(t2);                 // t3 = ((2*mu1_mu2 + C1).*(2*sigma12 + C2))
    t1 = mu1_2 + mu2_2 + C1;
    t2 = sigma1_2 + sigma2_2 + C2;
    t1 = t1.mul(t2);                 // t1 =((mu1_2 + mu2_2 + C1).*(sigma1_2 + sigma2_2 + C2))
    Mat ssim_map;
    divide(t3, t1, ssim_map);        // ssim_map =  t3./t1;
    Scalar mssim = mean(ssim_map);   // mssim = average of ssim map
    return mssim;
}
