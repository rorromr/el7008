#define _DEBUG

/* -------- Debug macros -------- */
#ifndef _DEBUG
#define PRINT_MAT(...)
#define PRINT(...)
#define STREAM(...)
#else
#define PRINT_MAT(...) printMat(__VA_ARGS__)
#define PRINT(...) printf(__VA_ARGS__)
#define STREAM(...) std::cout<<__VA_ARGS__<<std::endl
#endif

#define DEG2RAD 0.01745329251
#define NORM_FACTOR 0.06349363593424

#include <cstdio>
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>


void printMat(const cv::Mat &mat, const std::string &name = "M")
{
    std::cout << name << " = " << std::endl << " "  << mat << std::endl << std::endl;
}


//https://github.com/Itseez/opencv/blob/2.4/modules/contrib/src/facerec.cpp#L719
template <typename _Tp> static
inline void elbp_(cv::InputArray _src, cv::OutputArray _dst, int radius, int neighbors) {
    //get matrices
    cv::Mat src = _src.getMat();
    // allocate memory for result
    _dst.create(src.rows-2*radius, src.cols-2*radius, CV_32SC1);
    cv::Mat dst = _dst.getMat();
    // zero
    dst.setTo(0);
    for(int n=0; n<neighbors; n++) {
        // sample points
        float x = static_cast<float>(radius * cos(2.0*CV_PI*n/static_cast<float>(neighbors)));
        float y = static_cast<float>(-radius * sin(2.0*CV_PI*n/static_cast<float>(neighbors)));
        // relative indices
        int fx = static_cast<int>(floor(x));
        int fy = static_cast<int>(floor(y));
        int cx = static_cast<int>(ceil(x));
        int cy = static_cast<int>(ceil(y));
        // fractional part
        float ty = y - fy;
        float tx = x - fx;
        // set interpolation weights
        float w1 = (1 - tx) * (1 - ty);
        float w2 =      tx  * (1 - ty);
        float w3 = (1 - tx) *      ty;
        float w4 =      tx  *      ty;
        // iterate through your data
        for(int i=radius; i < src.rows-radius;i++) {
            for(int j=radius;j < src.cols-radius;j++) {
                // calculate interpolated value
                float t = static_cast<float>(w1*src.at<_Tp>(i+fy,j+fx) + w2*src.at<_Tp>(i+fy,j+cx) + w3*src.at<_Tp>(i+cy,j+fx) + w4*src.at<_Tp>(i+cy,j+cx));
                // floating point precision, so check some machine-dependent epsilon
                dst.at<int>(i-radius,j-radius) += ((t > src.at<_Tp>(i,j)) || (std::abs(t-src.at<_Tp>(i,j)) < std::numeric_limits<float>::epsilon())) << n;
            }
        }
    }
}



void printHelp(const char* name)
{
  std::cout << "Uso: "<< name << " imagen ground_truth" << std::endl;
  std::cout << "Ejemplo: $ "<< name <<" ./db/0024.jpg ./db/0024.bmp" << std::endl;
}

void t5( int, void* );
/* -------- Global params -------- */
std::string windowName = "t5 LBP";
// Theta
int thetaInt = 50;
std::string thetaTrackbar = "Theta";
int const thetaMax = 100;

cv::Mat input, ground_truth, gt_b, gt_gray, output, output_b; // Crear matriz de OpenCV



int main(int argc, char** argv)
{
  if( argc != 3)
  {
      printHelp(argv[0]);
      return 1;
  }
  std::string image_name(argv[1]);
  std::string gt_name(argv[2]);
  
  input = cv::imread(image_name); //Leer imagen
  ground_truth = cv::imread(gt_name); //Leer imagen

  if(input.empty()) // No encontro la imagen
  {
    std::cout << "Imagen '" << image_name << "' no encontrada" << std::endl;
    return 1; // Sale del programa anormalmente
  }
  if(ground_truth.empty()) // No encontro ground truth
  {
    std::cout << "Imagen Ground Truth'" << gt_name << "' no encontrada" << std::endl;
    return 1; // Sale del programa anormalmente
  }

  t5(0,0);
  std::cout << "Presiona ESC (GUI) o Ctrl+C para salir" << std::endl;

  while(true)
  {
    int c;
    c = cv::waitKey( 20 );
    if( (char)c == 27)
      { break; }
  }

  return 0;
}

void t5( int, void* )
{
  ;
}
