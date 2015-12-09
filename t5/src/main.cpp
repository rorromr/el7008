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

#include <cstdio>
#include <iostream>
#include <fstream>
#include <math.h>
// File utils
#include <boost/filesystem.hpp>
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

namespace fs = ::boost::filesystem;

// LBP code from https://github.com/Itseez/opencv/blob/2.4/modules/contrib/src/facerec.cpp
inline cv::Mat lbp(const cv::Mat& src)
{
  cv::Mat dst = cv::Mat::zeros(src.rows-2, src.cols-2, CV_8UC1);
  for(int i=1;i<src.rows-1;++i)
  {
      for(int j=1;j<src.cols-1;++j)
      {
          unsigned char center = src.at<unsigned char>(i,j);
          unsigned char code = 0;
          code |= (src.at<unsigned char>(i-1,j-1) >= center) << 7;
          code |= (src.at<unsigned char>(i-1,j) >= center) << 6;
          code |= (src.at<unsigned char>(i-1,j+1) >= center) << 5;
          code |= (src.at<unsigned char>(i,j+1) >= center) << 4;
          code |= (src.at<unsigned char>(i+1,j+1) >= center) << 3;
          code |= (src.at<unsigned char>(i+1,j) >= center) << 2;
          code |= (src.at<unsigned char>(i+1,j-1) >= center) << 1;
          code |= (src.at<unsigned char>(i,j-1) >= center) << 0;
          dst.at<unsigned char>(i-1,j-1) = code;
      }
  }
  return dst;
}

cv::Mat calcHist(const cv::Mat& src, int minVal=0, int maxVal=255, bool normed=false)
{
    cv::Mat result;
    // Establish the number of bins.
    int histSize = maxVal-minVal+1;
    // Set the ranges.
    float range[] = { static_cast<float>(minVal), static_cast<float>(maxVal+1) };
    const float* histRange = { range };
    // calc histogram
    cv::calcHist(&src, 1, 0, cv::Mat(), result, 1, &histSize, &histRange, true, false);
    // normalize
    if(normed)
    {
        result /= (int)src.total();
    }
    return result.reshape(1,1);
}

cv::Mat spatialHistogram(const cv::Mat& src, int numPatterns, int grid_x, int grid_y)
{
    // calculate LBP patch size
    int width = src.cols/grid_x;
    int height = src.rows/grid_y;
    // allocate memory for the spatial histogram
    cv::Mat result = cv::Mat::zeros(grid_x * grid_y, numPatterns, CV_32FC1);
    // return matrix with zeros if no data was given
    if(src.empty())
        return result.reshape(1,1);
    // initial result_row
    int resultRowIdx = 0;
    // iterate through grid
    for(int i = 0; i < grid_y; i++)
    {
        for(int j = 0; j < grid_x; j++)
        {
            // window
            cv::Mat src_cell = cv::Mat(src, cv::Range(i*height,(i+1)*height), cv::Range(j*width,(j+1)*width));
            cv::Mat cell_hist = calcHist(cv::Mat_<float>(src_cell), 0, (numPatterns-1), true);
            // copy to the result matrix
            cv::Mat result_row = result.row(resultRowIdx);
            cell_hist.reshape(1,1).convertTo(result_row, CV_32FC1);
            // increase row count in result matrix
            resultRowIdx++;
        }
    }
    // return result as reshaped feature vector
    return result.reshape(1,1);
}

class Face
{
  public:
    cv::Point2i left;
    cv::Point2i right;
    cv::Mat face;

    // Crop Constants
    static const int CROP_X = 40, CROP_UP = 50, CROP_DOWN = 70;

    // LBP Constants
    static const int LBP_RADIUS = 1, LBP_NEIGHBORS = 8;
    static const int LBP_GRID_X = 2, LBP_GRID_Y = 2;

    Face():
      left(0,0),
      right(0,0)
    {
    }

    Face(const std::string& file_name, const std::string& info_ext = std::string(".txt"),
      const std::string& img_ext = std::string(".jpg"))
    {
      // Open info file
      std::ifstream in((file_name+info_ext).c_str());
      // Fill info
      in >> *this;
      // Load image as CV_8UC1
      face = cv::imread((file_name+img_ext), CV_LOAD_IMAGE_GRAYSCALE);
    }

    void crop()
    {
      int x_mean = (left.x + right.x)/2.0;
      int y_mean = (left.y + right.y)/2.0;
      // Calc rectangle
      int x1 = std::max(x_mean - CROP_X, 0);
      int x2 = std::min(x_mean + CROP_X, face.cols - 1);
      int y1 = std::max(y_mean - CROP_UP, 0);
      int y2 = std::min(y_mean + CROP_DOWN, face.rows - 1);
      // Rectange width and height
      int w = x2-x1;
      int h = y2-y1;
      cv::Rect rect(x1, y1, w, h);

      // Crop face
      cv::Mat cropped(face, rect);
      cropped.copyTo(face);
    }

    cv::Mat getLBPHist()
    {
      // Calc LBP
      cv::Mat lbp_image = lbp(face);
      // Spatial histogram
      return spatialHistogram(lbp_image, 
        static_cast<int>(std::pow(2.0, static_cast<double>(LBP_NEIGHBORS))),
        LBP_GRID_X, LBP_GRID_Y);
    }

    cv::Mat getLBP()
    {
      return lbp(face);
    }

    void show(int delay = 20)
    {
      cv::imshow("Face", face);
      cv::waitKey(delay);
    }

    friend std::ostream &operator<< (std::ostream &output, const Face& info)
    { 
      output << "l:(" << info.left.x << "," << info.left.y << ") r:(" << info.right.x << "," << info.right.y << ")";
      return output;            
    }

    friend std::istream &operator>> (std::istream  &input, Face& info)
    { 
      input >> info.left.x >> info.left.y >> info.right.x >> info.right.y;
      return input;            
    }
};

void getHist(const fs::path& root, cv::Mat& hist)
{
  if(!fs::exists(root) || !fs::is_directory(root)) return;

  fs::recursive_directory_iterator it(root);
  fs::recursive_directory_iterator endit;

  while(it != endit)
  {
    if(fs::is_regular_file(*it))
    {
      std::string full_path = it->path().string();
      std::string file_name = full_path.substr(0,full_path.find_last_of("."));
      Face face(file_name);
      face.crop();
      hist.push_back(face.getLBPHist());
    }
    ++it;
  }
}

void getSet(int n_train, const cv::Mat& src, cv::Mat& train, cv::Mat& test)
{
  std::srand(std::time(0)); // Set seed
  int n = src.rows;
  if (n_train > n) return; // Check
  // Generate random permutation
  std::vector<int> idx(n);
  for (int i = 0; i < n; ++i) idx[i] = i;
  std::random_shuffle(idx.begin(), idx.end());
  // Fill data
  for (int i = 0; i < n_train; ++i)
    train.push_back(src.row(idx[i]));
  for (int i = n_train; i < n; ++i)
    test.push_back(src.row(idx[i]));
}

void printMat(const cv::Mat &mat, const std::string &name = "M")
{
    std::cout << name << " = " << std::endl << " "  << mat << std::endl << std::endl;
}


void printHelp(const char* name)
{
  std::cout << "Uso: " << std::endl;
  std::cout << "\tPara entrenar: " << name << " train ./db/female ./db/male" << std::endl;
  std::cout << "\tPara LBP: " << name << " lbp ./db/female/cache2335952.jpg" << std::endl;
  std::cout << "\tPara Clasificar: " << name << " test ./db/female/cache2335952.jpg" << std::endl;
}


int main(int argc, char** argv)
{
  if( argc > 3)
  {
      printHelp(argv[0]);
      return 1;
  }
  // LBP Option
  std::string option(argv[1]);
  if (option == "lbp")
  {
    std::string full_path(argv[2]);
    std::string file_name = full_path.substr(0,full_path.find_last_of("."));
    Face face(file_name);
    face.crop();
    std::cout << "Presione ESC para salir" << std::endl;
    cv::imshow("LBP",face.getLBP());
    face.show(0);
    return 0;
  }

  cv::SVM SVM;
  if (option == "test")
  {
    if (!fs::exists("svm.xml")){
      std::cout << "Archivo de entrenamiento 'svm.xml' no encontrado. Debe entrenar el clasificador." << std::endl;
      return 0;
    }
    std::string full_path(argv[2]);
    std::string file_name = full_path.substr(0,full_path.find_last_of("."));
    Face face(file_name);
    face.crop();
    cv::Mat result;
    SVM.load("svm.xml");
    SVM.predict(face.getLBPHist(), result);
    std::cout << (result.at<float>(0,0) > 0.9 ? "Mujer" : "Hombre") << std::endl;
    return 0;
  }

  std::cout << "Female DB: " << argv[2] << std::endl;
  std::cout << "Male DB: " << argv[3] << std::endl;
  
  // Get features
  cv::Mat female_feat, male_feat;
  getHist(fs::path(argv[2]), female_feat);
  getHist(fs::path(argv[3]), male_feat);

  // Size of train set
  int n_train_female = static_cast<int>(floor(0.7*female_feat.rows));
  int n_train_male = static_cast<int>(floor(0.7*male_feat.rows));

  // Get sets
  cv::Mat female_train_feat, male_train_feat;
  cv::Mat female_test_feat, male_test_feat;
  getSet(n_train_female, female_feat, female_train_feat, female_test_feat);
  getSet(n_train_male, male_feat, male_train_feat, male_test_feat);

  // Train data
  cv::Mat female_train_labels   = cv::Mat(female_train_feat.rows, 1, CV_32FC1, cv::Scalar(1.0));
  cv::Mat male_train_labels = cv::Mat(male_train_feat.rows, 1, CV_32FC1, cv::Scalar(0.0));
  std::cout << "Female label: 1" << std::endl;
  std::cout << "Male label: 0" << std::endl;

  cv::Mat train_labels, train_feat;
  cv::vconcat(female_train_feat, male_train_feat, train_feat);
  cv::vconcat(female_train_labels, male_train_labels, train_labels);
  std::cout << "Train data: (" << train_feat.rows << "," << train_feat.cols << ")" << std::endl;

  // SVM parameters
  cv::SVMParams params;
  params.svm_type = cv::SVM::C_SVC;
  params.kernel_type = cv::SVM::RBF;
  params.gamma = 3.5;
  params.C =  15;
  params.term_crit = cv::TermCriteria(CV_TERMCRIT_ITER, 10000, 1e-5);

  // Train SVM
  std::cout << "Training SVM..." << std::endl;
  SVM.train(train_feat, train_labels, cv::Mat(), cv::Mat(), params);

  // Test data
  cv::Mat female_results, male_results;
  SVM.predict(female_test_feat, female_results);
  SVM.predict(male_test_feat, male_results);
  SVM.save("svm.xml");
  
  // Results
  int tp = cv::sum(female_results)[0]; // True positives female
  int cp = female_results.rows; // Condition Positives
  int fn = cp - tp; // False negative
  float tpr = static_cast<float>(tp)/cp;

  int fp = cv::sum(male_results)[0]; // False positives
  int cn = male_results.rows; // Condition Negatives
  int tn = cn - fp; // True Negatives
  float fpr = static_cast<float>(fp)/cn;
  
  // Accuracy
  float acc = static_cast<float>(tp+tn)/(cp+cn);

  std::cout << "Results:" << std::endl;
  std::cout << "TPR: " << tpr << std::endl;
  std::cout << "FPR: " << fpr << std::endl;
  std::cout << "Accuracy: " << acc << std::endl;
  
  return 0;
}
