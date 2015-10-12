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

int main( int argc, char** argv)
{
  // Parameters
  float edgeThreshold = 2.0; // Binary threshold for edge detection


  Mat src, src_gray, grad;

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
  cvtColor(src, src_gray, CV_BGR2GRAY);   
  Mat input;
  src_gray.convertTo(input, CV_32F);

  // Apply gradient filter
  edgeDetector(src_gray, grad, edgeThreshold);

  // Show image
  imshow( "Test", grad );
  waitKey(500);

  return 0;
}
