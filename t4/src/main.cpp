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
    
    inline Gaussian(const Eigen::Vector3f& mean, const Eigen::Diagonal3f& cov, float weight):
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

void fillMoG(MoG& skin, MoG& nskin)
{
  // Skin MoG
  skin.add(Gaussian(Eigen::Vector3f(73.53,29.94,17.76),    Eigen::Diagonal3f(765.40,121.44,112.80), 0.0294));
  skin.add(Gaussian(Eigen::Vector3f(249.71,233.94,217.49), Eigen::Diagonal3f(39.94,154.44,396.05),  0.0331));
  skin.add(Gaussian(Eigen::Vector3f(161.68,116.25,96.95),  Eigen::Diagonal3f(291.03,60.48,162.85),  0.0654));
  skin.add(Gaussian(Eigen::Vector3f(186.07,136.62,114.40), Eigen::Diagonal3f(274.95,64.60,198.27),  0.0756));
  skin.add(Gaussian(Eigen::Vector3f(189.26,98.37,51.18),   Eigen::Diagonal3f(633.18,222.40,250.69), 0.0554));
  skin.add(Gaussian(Eigen::Vector3f(247.00,152.20,90.84),  Eigen::Diagonal3f(65.23,691.53,609.92),  0.0314));
  skin.add(Gaussian(Eigen::Vector3f(150.10,72.66,37.76),   Eigen::Diagonal3f(408.63,200.77,257.57), 0.0454));
  skin.add(Gaussian(Eigen::Vector3f(206.85,171.09,156.34), Eigen::Diagonal3f(530.08,155.08,572.79), 0.0469));
  skin.add(Gaussian(Eigen::Vector3f(212.78,152.82,120.04), Eigen::Diagonal3f(160.57,84.52,243.90),  0.0956));
  skin.add(Gaussian(Eigen::Vector3f(234.87,175.43,138.94), Eigen::Diagonal3f(163.80,121.57,279.22), 0.0763));
  skin.add(Gaussian(Eigen::Vector3f(151.19,97.74,74.59),   Eigen::Diagonal3f(425.40,73.56,175.11),  0.1100));
  skin.add(Gaussian(Eigen::Vector3f(120.52,77.55,59.82),   Eigen::Diagonal3f(330.45,70.34,151.82),  0.0676));
  skin.add(Gaussian(Eigen::Vector3f(192.20,119.62,82.32),  Eigen::Diagonal3f(152.76,92.14,259.15),  0.0755));
  skin.add(Gaussian(Eigen::Vector3f(214.29,136.08,87.24),  Eigen::Diagonal3f(204.90,140.17,270.19), 0.0500));
  skin.add(Gaussian(Eigen::Vector3f(99.57,54.33,38.06),    Eigen::Diagonal3f(448.13,90.18,151.29),  0.0667));
  skin.add(Gaussian(Eigen::Vector3f(238.88,203.08,176.91), Eigen::Diagonal3f(178.38,156.27,404.99), 0.0749));

  // Non skin MoG
  nskin.add(Gaussian(Eigen::Vector3f(254.37,254.41,253.82), Eigen::Diagonal3f(2.77,2.81,5.46),         0.0637));
  nskin.add(Gaussian(Eigen::Vector3f(9.39,8.09,8.52),       Eigen::Diagonal3f(46.84,33.59,32.48),      0.0516));
  nskin.add(Gaussian(Eigen::Vector3f(96.57,96.95,91.53),    Eigen::Diagonal3f(280.69,156.79,436.58),   0.0864));
  nskin.add(Gaussian(Eigen::Vector3f(160.44,162.49,159.06), Eigen::Diagonal3f(355.98,115.89,591.24),   0.0636));
  nskin.add(Gaussian(Eigen::Vector3f(74.98,63.23,46.33),    Eigen::Diagonal3f(414.84,245.95,361.27),   0.0747));
  nskin.add(Gaussian(Eigen::Vector3f(121.83,60.88,18.31),   Eigen::Diagonal3f(2502.24,1383.53,237.18), 0.0365));
  nskin.add(Gaussian(Eigen::Vector3f(202.18,154.88,91.04),  Eigen::Diagonal3f(957.42,1766.94,1582.52), 0.0349));
  nskin.add(Gaussian(Eigen::Vector3f(193.06,201.93,206.55), Eigen::Diagonal3f(562.88,190.23,447.28),   0.0649));
  nskin.add(Gaussian(Eigen::Vector3f(51.88,57.14,61.55),    Eigen::Diagonal3f(344.11,191.77,433.40),   0.0656));
  nskin.add(Gaussian(Eigen::Vector3f(30.88,26.84,25.32),    Eigen::Diagonal3f(222.07,118.65,182.41),   0.1189));
  nskin.add(Gaussian(Eigen::Vector3f(44.97,85.96,131.95),   Eigen::Diagonal3f(651.32,840.52,963.67),   0.0362));
  nskin.add(Gaussian(Eigen::Vector3f(236.02,236.27,230.70), Eigen::Diagonal3f(225.03,117.29,331.95),   0.0849));
  nskin.add(Gaussian(Eigen::Vector3f(207.86,191.20,164.12), Eigen::Diagonal3f(494.04,237.69,533.52),   0.0368));
  nskin.add(Gaussian(Eigen::Vector3f(99.83,148.11,188.17),  Eigen::Diagonal3f(955.88,654.95,916.70),   0.0389));
  nskin.add(Gaussian(Eigen::Vector3f(135.06,131.92,123.10), Eigen::Diagonal3f(350.35,130.30,388.43),   0.0943));
  nskin.add(Gaussian(Eigen::Vector3f(135.96,103.89,66.88),  Eigen::Diagonal3f(806.44,642.20,350.36),   0.0477));
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
