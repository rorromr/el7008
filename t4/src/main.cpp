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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <Eigen/Core>

using namespace std;
using namespace cv;

void printMat(const Mat &mat, const string &name = "M")
{
    std::cout << name << " = " << std::endl << " "  << mat << std::endl << std::endl;
}
void t4( int, void* );

namespace Eigen
{
  typedef DiagonalMatrix<float, 3, 3> Diagonal3f;  
}

/* Gaussian distribution with diagonal covariance */
class Gaussian
{
  public:
    Eigen::Vector3f mean_;
    Eigen::Diagonal3f cov_inv_;
    float factor_;
    
    Gaussian(const Eigen::Vector3f& mean, const Eigen::Diagonal3f& cov, float weight):
      mean_(mean),
      cov_inv_(cov.inverse()),
      factor_(weight*NORM_FACTOR*cov.diagonal().norm()) {};

    float evaluate(const Eigen::Vector3f& x) const
    {
      return factor_*exp(-0.5*(x-mean_).transpose()*cov_inv_*(x-mean_));
    }
};

/* Mixture of Gaussians */
class MoG
{
  public:
    std::vector<Gaussian> gaussians_;
  
  MoG(const std::size_t size)
  {
    gaussians_.reserve(size);
  }

  void add(const Gaussian& g)
  {
    gaussians_.push_back(g);
  }

  float evaluate(const Eigen::Vector3f& x) const
  {
    float val = 0;
    std::vector<Gaussian>::const_iterator g(gaussians_.begin()), end(gaussians_.end());
    for (; g != end; ++g)
    {
      val += g->evaluate(x);
    }
    return val;
  } 
};

void evaluateImage(const cv::Mat& input, cv::Mat& output)
{
  ;
}




void printHelp(const char* name)
{
    cout << "Uso: "<< name << " image image_ref" << endl;
    cout << "Ejemplo: $ "<< name <<" ice1.jpg ice2.jpg" <<endl;
}

Mat input1, input2; // Crear matriz de OpenCV

int main(int argc, char** argv)
{
  if( argc != 3)
  {
      printHelp(argv[0]);
      return 1;
  }
  string image_name1(argv[1]);
  string image_name2(argv[2]);
  
  input1 = imread(image_name1); //Leer imagen
  input2 = imread(image_name2); //Leer imagen

  if(input1.empty() || input2.empty()) // No encontro la imagen
  {
    cout<<"Imagen '"<< image_name1 <<"' no encontrada"<<endl;
    return 1; // Sale del programa anormalmente
  }

  t4(0,0);
  cout << "Presiona ESC (GUI) o Ctrl+C para salir" << endl;

  while(true)
  {
    int c;
    c = waitKey( 20 );
    if( (char)c == 27)
      { break; }
  }

  return 0;
}

void t4( int, void* )
{
}
