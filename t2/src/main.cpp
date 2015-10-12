#define _DEBUG

// Print functions for debug
#ifndef _DEBUG
#define DEBUG_PRINT(...)
#else
#define DEBUG_PRINT(...) printMat(__VA_ARGS__)
#endif

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

void printMat(const Mat &mat, const string &name = "M")
{
    cout << name << " = " << endl << " "  << mat << endl << endl;
}

void edgeDetector(const Mat &input, Mat &output, float threshold = 1.0)
{
  // Generate grad_x and grad_y
  Mat grad_x, grad_y;
  Mat abs_grad_x, abs_grad_y;

  // Gradient X
  Mat kx, ky;
  getDerivKernels( kx, ky, 1, 0, 3, false, CV_32F); // Derivate x
  sepFilter2D( input, grad_x, CV_32F, kx, ky, Point(-1, -1), 0, BORDER_DEFAULT);
  DEBUG_PRINT(kx, "kx");
  convertScaleAbs( grad_x, abs_grad_x );

  // Gradient Y
  getDerivKernels( kx, ky, 0, 1, 3, false, CV_32F); // Derivate y
  sepFilter2D( input, grad_y, CV_32F, kx, ky, Point(-1, -1), 0, BORDER_DEFAULT);
  DEBUG_PRINT(ky, "ky");
  convertScaleAbs( grad_y, abs_grad_y );

  // Total Gradient (approximate)
  addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, output );

  // Median
  float mean = cv::mean(output)[0];
  cv::threshold(output, output, threshold*mean, 255,  cv::THRESH_BINARY);
}

void t2( int, void* );

Mat input, output;
string windowName = "t2";

/* -------- Parameters -------- */

// Edge threshold ET
string ETtrackbar = "Edge Thres.";
int ETvalue = 10; // Binary threshold for edge detection
int const ETmax = 255;

int main( int argc, char** argv)
{
  

  Mat src, src_gray;

  // Load an image
  src = imread( argv[1] );
  // Check
  if( src.empty() )
  { 
    cout << "Imagen no encontrada" << endl;
    return -1;
  }

  // Filtro gaussiano
  GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );

  // Convert it to gray
  cvtColor(src, input, CV_BGR2GRAY);   

  namedWindow( windowName, CV_WINDOW_AUTOSIZE );
  createTrackbar( ETtrackbar, windowName, &ETvalue, ETmax, t2);
  
  t2(0,0);

  while(true)
  {
    int c;
    c = waitKey( 20 );
    if( (char)c == 27)
      { break; }
   }

  return 0;
}

void t2( int, void* )
{
  // Apply edge detector
  float edgeThreshold = ETvalue*5.0/ETmax;
  edgeDetector(input, output, edgeThreshold);

  // Show image
  imshow( windowName, output );
}
