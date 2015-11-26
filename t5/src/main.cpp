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
#include <Eigen/Core>


void printMat(const cv::Mat &mat, const std::string &name = "M")
{
    std::cout << name << " = " << std::endl << " "  << mat << std::endl << std::endl;
}

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
  
  MoG(const std::size_t size = 16)
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

typedef struct ROCPoint
{
  float theta;
  float tpr;
  float fpr;
  inline ROCPoint(float t): theta(t), tpr(0.0), fpr(0.0) {};
} ROCPoint;

class ROC
{
  public:
    std::vector<ROCPoint> point;
    
    ROC(std::size_t size, float theta_min, float theta_max)
    {
      point.reserve(size);
      float dtheta = (theta_max - theta_min)/size;
      for (std::size_t i = 0; i < size; ++i)
      {
        point.push_back(ROCPoint(dtheta*i));
      }
    }

    void save(const char* file_name) const
    {
      std::ofstream file(file_name);
      if (file.is_open())
      {
        for (std::vector<ROCPoint>::const_iterator i = point.begin(); i != point.end(); ++i)
        {
          file << i->theta << "," << i->tpr << "," << i->fpr << "\n";
        }
        file.close();
        std::cout << "ROC saved: '" << file_name << "'" << std::endl;
      }
      else std::cout << "Unable save file" << std::endl;
    }
};


void imageProc(const MoG& skin_mog, const MoG& nskin_mog, const cv::Mat& input, cv::Mat& output)
{
  output = cv::Mat::zeros(input.rows,input.cols,CV_32F);

  for (int i = 0; i < input.rows; ++i) {
    for (int j = 0; j < input.cols; ++j) {

      const cv::Vec3b intensity = input.at<cv::Vec3b>(i, j);
      // BGR
      const Eigen::Vector3f pixel(intensity.val[2], intensity.val[1], intensity.val[0]);
      output.at<float>(i,j) = skin_mog.evaluate(pixel)/nskin_mog.evaluate(pixel);

    }
  }
}

void evaluate(const cv::Mat& input, const cv::Mat& gt, unsigned int cond_pos, unsigned int cond_neg, ROC& roc)
{
  for (std::vector<ROCPoint>::iterator p = roc.point.begin(); p != roc.point.end(); ++p)
  {
    for (int i = 0; i < input.rows; ++i)
    {
      for (int j = 0; j < input.cols; ++j)
      {
        bool skin_test = input.at<float>(i,j) > p->theta;
        bool skin_gt = gt.at<unsigned char>(i,j) > 0;
        if (skin_test && skin_gt) ++(p->tpr);
        else if (skin_test && !skin_gt) ++(p->fpr);
      }
    }
    p->tpr /= cond_pos;
    p->fpr /= cond_neg;
  }
}  

void fillMoG(MoG& skin_mog, MoG& nskin_mog)
{
  // Skin MoG
  skin_mog.add(Gaussian(Eigen::Vector3f(73.53,29.94,17.76),    Eigen::Diagonal3f(765.40,121.44,112.80), 0.0294));
  skin_mog.add(Gaussian(Eigen::Vector3f(249.71,233.94,217.49), Eigen::Diagonal3f(39.94,154.44,396.05),  0.0331));
  skin_mog.add(Gaussian(Eigen::Vector3f(161.68,116.25,96.95),  Eigen::Diagonal3f(291.03,60.48,162.85),  0.0654));
  skin_mog.add(Gaussian(Eigen::Vector3f(186.07,136.62,114.40), Eigen::Diagonal3f(274.95,64.60,198.27),  0.0756));
  skin_mog.add(Gaussian(Eigen::Vector3f(189.26,98.37,51.18),   Eigen::Diagonal3f(633.18,222.40,250.69), 0.0554));
  skin_mog.add(Gaussian(Eigen::Vector3f(247.00,152.20,90.84),  Eigen::Diagonal3f(65.23,691.53,609.92),  0.0314));
  skin_mog.add(Gaussian(Eigen::Vector3f(150.10,72.66,37.76),   Eigen::Diagonal3f(408.63,200.77,257.57), 0.0454));
  skin_mog.add(Gaussian(Eigen::Vector3f(206.85,171.09,156.34), Eigen::Diagonal3f(530.08,155.08,572.79), 0.0469));
  skin_mog.add(Gaussian(Eigen::Vector3f(212.78,152.82,120.04), Eigen::Diagonal3f(160.57,84.52,243.90),  0.0956));
  skin_mog.add(Gaussian(Eigen::Vector3f(234.87,175.43,138.94), Eigen::Diagonal3f(163.80,121.57,279.22), 0.0763));
  skin_mog.add(Gaussian(Eigen::Vector3f(151.19,97.74,74.59),   Eigen::Diagonal3f(425.40,73.56,175.11),  0.1100));
  skin_mog.add(Gaussian(Eigen::Vector3f(120.52,77.55,59.82),   Eigen::Diagonal3f(330.45,70.34,151.82),  0.0676));
  skin_mog.add(Gaussian(Eigen::Vector3f(192.20,119.62,82.32),  Eigen::Diagonal3f(152.76,92.14,259.15),  0.0755));
  skin_mog.add(Gaussian(Eigen::Vector3f(214.29,136.08,87.24),  Eigen::Diagonal3f(204.90,140.17,270.19), 0.0500));
  skin_mog.add(Gaussian(Eigen::Vector3f(99.57,54.33,38.06),    Eigen::Diagonal3f(448.13,90.18,151.29),  0.0667));
  skin_mog.add(Gaussian(Eigen::Vector3f(238.88,203.08,176.91), Eigen::Diagonal3f(178.38,156.27,404.99), 0.0749));

  // Non skin MoG
  nskin_mog.add(Gaussian(Eigen::Vector3f(254.37,254.41,253.82), Eigen::Diagonal3f(2.77,2.81,5.46),         0.0637));
  nskin_mog.add(Gaussian(Eigen::Vector3f(9.39,8.09,8.52),       Eigen::Diagonal3f(46.84,33.59,32.48),      0.0516));
  nskin_mog.add(Gaussian(Eigen::Vector3f(96.57,96.95,91.53),    Eigen::Diagonal3f(280.69,156.79,436.58),   0.0864));
  nskin_mog.add(Gaussian(Eigen::Vector3f(160.44,162.49,159.06), Eigen::Diagonal3f(355.98,115.89,591.24),   0.0636));
  nskin_mog.add(Gaussian(Eigen::Vector3f(74.98,63.23,46.33),    Eigen::Diagonal3f(414.84,245.95,361.27),   0.0747));
  nskin_mog.add(Gaussian(Eigen::Vector3f(121.83,60.88,18.31),   Eigen::Diagonal3f(2502.24,1383.53,237.18), 0.0365));
  nskin_mog.add(Gaussian(Eigen::Vector3f(202.18,154.88,91.04),  Eigen::Diagonal3f(957.42,1766.94,1582.52), 0.0349));
  nskin_mog.add(Gaussian(Eigen::Vector3f(193.06,201.93,206.55), Eigen::Diagonal3f(562.88,190.23,447.28),   0.0649));
  nskin_mog.add(Gaussian(Eigen::Vector3f(51.88,57.14,61.55),    Eigen::Diagonal3f(344.11,191.77,433.40),   0.0656));
  nskin_mog.add(Gaussian(Eigen::Vector3f(30.88,26.84,25.32),    Eigen::Diagonal3f(222.07,118.65,182.41),   0.1189));
  nskin_mog.add(Gaussian(Eigen::Vector3f(44.97,85.96,131.95),   Eigen::Diagonal3f(651.32,840.52,963.67),   0.0362));
  nskin_mog.add(Gaussian(Eigen::Vector3f(236.02,236.27,230.70), Eigen::Diagonal3f(225.03,117.29,331.95),   0.0849));
  nskin_mog.add(Gaussian(Eigen::Vector3f(207.86,191.20,164.12), Eigen::Diagonal3f(494.04,237.69,533.52),   0.0368));
  nskin_mog.add(Gaussian(Eigen::Vector3f(99.83,148.11,188.17),  Eigen::Diagonal3f(955.88,654.95,916.70),   0.0389));
  nskin_mog.add(Gaussian(Eigen::Vector3f(135.06,131.92,123.10), Eigen::Diagonal3f(350.35,130.30,388.43),   0.0943));
  nskin_mog.add(Gaussian(Eigen::Vector3f(135.96,103.89,66.88),  Eigen::Diagonal3f(806.44,642.20,350.36),   0.0477));
}


void printHelp(const char* name)
{
  std::cout << "Uso: "<< name << " imagen ground_truth" << std::endl;
  std::cout << "Ejemplo: $ "<< name <<" ./db/0024.jpg ./db/0024.bmp" << std::endl;
}

void t4( int, void* );
/* -------- Global params -------- */
std::string windowName = "t4 MoG: Skin";
// Theta
int thetaInt = 50;
std::string thetaTrackbar = "Theta";
int const thetaMax = 100;

cv::Mat input, ground_truth, gt_b, gt_gray, output, output_b; // Crear matriz de OpenCV
MoG skin, nskin;
ROC roc(100, 0.01, 2.0);

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
  // Ground Truth threshold
  cv::cvtColor(ground_truth, gt_gray, CV_BGR2GRAY);
  cv::threshold(gt_gray, gt_b, 245, 255, 1);
  unsigned int cond_pos = cv::countNonZero(gt_b);
  unsigned int cond_neg = gt_b.cols*gt_b.rows-cond_pos;
  STREAM("Cond Pos: " << cond_pos);
  STREAM("Cond Neg: " << cond_neg);
  // Fill MoG params
  fillMoG(skin, nskin);

  // Window
  cv::namedWindow( windowName, CV_WINDOW_AUTOSIZE );
  cv::createTrackbar( thetaTrackbar, windowName, &thetaInt, thetaMax, t4);

  imageProc( skin, nskin, input, output);
  evaluate(output, gt_b, cond_pos, cond_neg, roc);
  // get name
  std::string roc_name = gt_name.substr(0,gt_name.find_last_of(".")) + "_roc.cvs";
  roc.save(roc_name.c_str());

  t4(0,0);
  cv::imshow("t4 MoG: Ground Truth", gt_b);
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

void t4( int, void* )
{
  float theta = 0.02*thetaInt;
  std::cout << "Using theta = " << theta << std::endl;
  cv::threshold(output, output_b, theta, 3.0, 1);  
  cv::imshow( windowName, output_b);
}
