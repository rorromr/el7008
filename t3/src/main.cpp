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

#include <cstdio>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>

using namespace std;
using namespace cv;

void printMat(const Mat &mat, const string &name = "M")
{
    cout << name << " = " << endl << " "  << mat << endl << endl;
}

void genTransform(DMatch match, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2, double &e, double &theta, double &tx, double &ty)
{
  // Prueba: keypoints2, referencia: keypoints1

  // Scale factor
  e = ((double)keypoints2[match.queryIdx].size)/keypoints1[match.trainIdx].size;
  
  // Transform angle
  theta = keypoints2[match.queryIdx].angle - keypoints1[match.trainIdx].angle;

  // Transform traslation
  double x_ref = keypoints1[match.trainIdx].pt.x;
  double y_ref = keypoints1[match.trainIdx].pt.y;
  tx = keypoints2[match.queryIdx].pt.x - e*(x_ref*cos(theta*DEG2RAD)-y_ref*sin(theta*DEG2RAD));
  ty = keypoints2[match.queryIdx].pt.y - e*(x_ref*sin(theta*DEG2RAD)+y_ref*cos(theta*DEG2RAD));
}

int computeConsensus(vector<DMatch> &matches, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2, vector<int> &selected, double e, double theta, double tx, double ty)
{
    int cons = 0;
    selected.clear();
    for (int i=0; i<(int)matches.size(); i++)
    {
      double x1 = keypoints1[matches[i].trainIdx].pt.x;
      double y1 = keypoints1[matches[i].trainIdx].pt.y;
      double x2 = keypoints2[matches[i].queryIdx].pt.x;
      double y2 = keypoints2[matches[i].queryIdx].pt.y;
      
      // Estimate with proyection
      double x2t = tx + e*(x1*cos(theta*DEG2RAD) - y1*sin(theta*DEG2RAD));
      double y2t = ty + e*(x1*sin(theta*DEG2RAD) + y1*cos(theta*DEG2RAD));
      
      // Proyection error
      double ex = (x2t-x2)*(x2t-x2) + (y2t-y2)*(y2t-y2);
      if (ex < 400)
      {
        selected.push_back(i);
        cons++;
      }
    }
    return cons;
}

void ransac(vector<DMatch> &matches, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2, vector<DMatch> &accepted)
{
  vector<int> selected;
  double e, theta, tx, ty;
  
  for (int i = 0; i < 400; ++i)
  {
    int ind = rand() % matches.size();
    genTransform(matches[ind], keypoints1, keypoints2, e, theta, tx, ty);
    int consensus = computeConsensus(matches, keypoints1, keypoints2, selected, e, theta, tx, ty);
    if (consensus > 30)
    {
      for (int i=0; i<(int)selected.size(); i++)
        accepted.push_back(matches[selected[i]]);
    }
  }
}

void printHelp(const char* name)
{
    cout << "Uso: "<< name << " image image_ref" << endl;
    cout << "Ejemplo: $ "<< name <<" ice1.jpg ice2.jpg" <<endl;
}

int main(int argc, char** argv)
{
  if( argc != 3)
  {
      printHelp(argv[0]);
      return 1;
  }
  string image_name1(argv[1]);
  string image_name2(argv[2]);
  
  Mat input1, input2; // Crear matriz de OpenCV
  input1 = imread(image_name1); //Leer imagen
  input2 = imread(image_name2); //Leer imagen

  if(input1.empty() || input2.empty()) // No encontro la imagen
  {
    cout<<"Imagen no encontrada"<<endl;
    return 1; // Sale del programa anormalmente
  }

  SurfFeatureDetector detector;
  vector<KeyPoint> keypoints1;
  detector.detect(input1, keypoints1);

  vector<KeyPoint> keypoints2;
  detector.detect(input2, keypoints2);

  SurfDescriptorExtractor extractor;
  Mat descriptors1, descriptors2;
  extractor.compute(input1, keypoints1, descriptors1);
  extractor.compute(input2, keypoints2, descriptors2);

  BFMatcher matcher(NORM_L2);
  vector<DMatch> matches;
  matcher.match(descriptors2, descriptors1, matches);

  vector<DMatch> accepted;
  ransac(matches, keypoints1, keypoints2, accepted);

  // drawing the results
  Mat output;
  drawKeypoints(input1, keypoints1, output);
  imshow("keypoints1", output);
  drawKeypoints(input2, keypoints2, output);
  imshow("keypoints2", output);

  imwrite("sift_result.jpg", output);
  namedWindow("matches", 1);
  Mat img_matches;
  drawMatches(input2, keypoints2, input1, keypoints1, matches, img_matches);
  imshow("matches", img_matches);
  imwrite("matches.jpg", img_matches);

  Mat img_accepted;
  drawMatches(input2, keypoints2, input1, keypoints1, accepted, img_matches);
  imshow("accepted", img_matches);
  imwrite("accepted.jpg", img_matches);

  cout << "Presione ENTER en una ventana o CTRL-C para salir" << endl;
  waitKey(0);

  return 0;
}
