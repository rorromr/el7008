//#define _DEBUG

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
template <typename _Tp>
inline cv::Mat lbp(const cv::Mat& src, int radius, int neighbors) {
    // Result
    cv::Mat dst(src.rows-2*radius, src.cols-2*radius, CV_32SC1);
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
    for(int i = 0; i < grid_y; i++) {
        for(int j = 0; j < grid_x; j++) {
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

    void getLBPHist(cv::Mat& histograms)
    {
      // Calc LBP
       cv::Mat lbp_image = lbp<unsigned char>(face, LBP_RADIUS, LBP_NEIGHBORS);
      // Spatial histogram
      cv::Mat p = spatialHistogram(lbp_image, 
        static_cast<int>(std::pow(2.0, static_cast<double>(LBP_NEIGHBORS))),
        LBP_GRID_X, LBP_GRID_Y);
      histograms.push_back(p);
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
      face.getLBPHist(hist);
      #ifdef _DEBUG
        face->show();
        STREAM(*face);
      #endif
    }
    ++it;
  }
}

void printMat(const cv::Mat &mat, const std::string &name = "M")
{
    std::cout << name << " = " << std::endl << " "  << mat << std::endl << std::endl;
}


void printHelp(const char* name)
{
  std::cout << "Uso: "<< name << " train ./db/female ./db/male" << std::endl;
}


int main(int argc, char** argv)
{
  if( argc != 3)
  {
      printHelp(argv[0]);
      return 1;
  }
  std::cout << "Female DB: " << argv[1] << std::endl;
  std::cout << "Male DB: " << argv[2] << std::endl;
  
  // Get features
  cv::Mat male_feat, female_feat;
  getHist(fs::path(argv[1]), female_feat);
  getHist(fs::path(argv[2]), male_feat);

  // Label
  cv::Mat female_labels   = cv::Mat(female_feat.rows, 1, CV_32FC1, cv::Scalar(1.0));
  cv::Mat male_labels = cv::Mat(male_feat.rows, 1, CV_32FC1, cv::Scalar(0.0));
  std::cout << "Female label: 1" << std::endl;
  std::cout << "Male label: 0" << std::endl;

  // Train data
  cv::Mat train_labels, train_feat;
  cv::vconcat(female_feat, male_feat, train_feat);
  cv::vconcat(male_labels, female_labels, train_labels);
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
  cv::SVM SVM;
  SVM.train(train_feat, train_labels, cv::Mat(), cv::Mat(), params);

  // Test
  cv::Mat test_male_results, test_female_results;
  SVM.predict(male_feat, test_male_results);
  SVM.predict(female_feat, test_female_results);

  // display results
  int s_male   = cv::sum(test_male_results  )[0];
  int s_female = cv::sum(test_female_results)[0];
  int len_male   = test_male_results.rows;
  int len_female = test_female_results.rows;
  int n_correct_classifications = s_male + (len_female-s_female);
  float rec_rate = n_correct_classifications/(len_male + len_female + 0.0);
  printf("\nBy genre:\n");
  printf(" - Male   (%f) - (%d/%d)\n",(0.0+s_male)/len_male,s_male,len_male);
  printf(" - Female (%f) - (%d/%d)\n",(0.0 + len_female-s_female)/len_female,(len_female-s_female),len_female);
  printf("Summary:\n");
  printf(" - class. rate (%f) - (%d/%d)\n",rec_rate,n_correct_classifications,len_male + len_female);
  printf(" - error. rate (%f) - (%d/%d)\n",1.0-rec_rate,len_male+len_female - n_correct_classifications,len_male + len_female);

  return 0;
}
