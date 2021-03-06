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

#define DEG2RAD 0.01745329251f

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cstdio>

using namespace cv;
using namespace std;

/* -------- Structs -------- */
typedef struct
{
  int acc;
  float r;
  float theta;
} HoughLine;

struct HoughLineComparator {
  bool operator() (const HoughLine &l1, const HoughLine &l2)
  {
    return l1.acc > l2.acc;
  }
};

typedef struct
{
  int acc;
  float a;
  float b;
  float r;
} HoughCircle;

struct HoughCircleComparator {
  bool operator() (const HoughCircle &c1, const HoughCircle &c2)
  {
    return c1.acc > c2.acc;
  }
};

/* -------- Main functions -------- */

void printMat(const Mat &mat, const string &name = "M")
{
    cout << name << " = " << endl << " "  << mat << endl << endl;
}

void edgeDetector(const Mat &input, Mat &output, float th = 1.0)
{
  // Generate grad_x and grad_y
  Mat grad_x, grad_y, grad;
  Mat abs_grad_x, abs_grad_y;

  // Gradient X
  Mat kx, ky;
  getDerivKernels( kx, ky, 1, 0, 3, false, CV_32F); // Derivate x
  sepFilter2D( input, grad_x, CV_32F, kx, ky, Point(-1, -1), 0, BORDER_DEFAULT);
  PRINT_MAT(kx, "kx");
  convertScaleAbs( grad_x, abs_grad_x );

  // Gradient Y
  getDerivKernels( kx, ky, 0, 1, 3, false, CV_32F); // Derivate y
  sepFilter2D( input, grad_y, CV_32F, kx, ky, Point(-1, -1), 0, BORDER_DEFAULT);
  PRINT_MAT(ky, "ky");
  convertScaleAbs( grad_y, abs_grad_y );

  // Total Gradient (approximate)
  addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

  // Mean
  float mean = cv::mean(grad)[0];
  output = Mat(grad.rows, grad.cols, CV_8UC1);
  threshold(grad, output, th*mean, 255,  cv::THRESH_BINARY);
}

/* -------- Hough lines detector -------- */
void houghLines(const Mat &input, Mat &hough, vector<HoughLine> &lines, int num_lines = 10, int theta_bin = 380, int r_bin = 200)
{
  // Image size
  int width = input.cols;
  int height = input.rows;

  // Hough accumulator
  // Max R size
  int rmax = (int) round(sqrt(width*width + height*height));
  
  float dr = 2.0*rmax/r_bin;
  float dtheta = 180.0/theta_bin*DEG2RAD;
  
  // Precalc
  float sin_lut[theta_bin];
  float cos_lut[theta_bin];
  float theta_count = -90.0*DEG2RAD;
  for (int i = 0; i < theta_bin; ++i, theta_count+=dtheta)
  {
    sin_lut[i]=sin(theta_count)/dr;
    cos_lut[i]=cos(theta_count)/dr;
  }

  // Create accumulator with extra rows and cols
  int** acc = new int*[r_bin+2];
  int i, j;
  for( i = 0; i < r_bin+2; ++i)
  {
    acc[i] = new int[theta_bin+2];
    for( j = 0; j < theta_bin+2; ++j)
    {
        acc[i][j] = 0;
    }
  }
  
  // Hough Transform
  for( j = 0; j < height; ++j)
  {
    for ( i = 0; i < width; ++i)
    {
      // Check if the point is an edge
      if (input.at<unsigned char>(j,i) > 0)
      {
        for(int theta_i = 0; theta_i < theta_bin; ++theta_i)
        {
          int r_i = (int) round(i*cos_lut[theta_i]+j*sin_lut[theta_i]);
          r_i += (r_bin-1)/2;

          r_i = min(max(r_i, 1), r_bin);
          ++acc[r_i][theta_i];
        }
      }
    }
  }

  // Hough image for visualization
  
  // Max value
  int maxv = 0;
  for (i = 0; i < r_bin+2; ++i)
  {
      for (j = 0; j < theta_bin+2; ++j)
      {
          maxv = max(maxv,acc[i][j]);
      }
  }

  // Create Hough Image
  hough = Mat(r_bin, theta_bin, CV_8UC1);

  for (i = 0; i < r_bin; ++i)
  {
    for (j = 0; j < theta_bin; ++j)
    {
        hough.at<unsigned char>(i,j) = (unsigned char)(acc[i][j]*255.0/maxv);
    }
  }

  // Peaks
  vector<HoughLine> peaks;
  float r_offset = -rmax;
  float theta_offset = -90.0*DEG2RAD;
  PRINT("r_offset %.3f, theta_offset %.3f\n", r_offset, theta_offset);
  const int votes_threshold = 25;
  const int neigh_threshold = 20;
  for (i = 1; i < r_bin; ++i)
  {
    for (j = 1; j < theta_bin; ++j)
    {
      int votes = acc[i][j];
      if ( votes > votes_threshold && 
        // Compare with neighborhood         
        votes > neigh_threshold+acc[i-1][j] &&
        votes > neigh_threshold+acc[i-1][j+1] &&
        votes > neigh_threshold+acc[i-1][j-1] &&

        votes > neigh_threshold+acc[i][j+1] &&
        votes > neigh_threshold+acc[i][j-1] &&
        
        votes > neigh_threshold+acc[i+1][j] &&
        votes > neigh_threshold+acc[i+1][j+1] &&
        votes > neigh_threshold+acc[i+1][j-1])
      {
        HoughLine l;
        l.acc = votes;
        l.r = (i+1)*dr + r_offset;
        l.theta = (j+1)*dtheta + theta_offset;
        peaks.push_back(l);
      }
    }
  }
  STREAM("Peaks: " << peaks.size());
  STREAM(" with th.: " << votes_threshold << endl);

  // Sort using votes
  HoughLineComparator comparator;
  sort(peaks.begin(), peaks.end(), comparator);
  // Add to lines
  num_lines = min(static_cast<size_t>(num_lines), peaks.size());
  for (i = 0; i < num_lines; ++i)
  {
    lines.push_back(peaks[i]);
  }

  for( i = 0; i < r_bin+2; ++i)
  {
    delete[] acc[i];
  }
  delete[] acc;
  STREAM("Found: " << num_lines);
}

void houghCircles(const Mat &input, Mat &hough, vector<HoughCircle> &circles, 
  int num_circles = 10, int a_bin = 15, int b_bin = 15, int r_bin = 15)
{
  // Image size
  int width = input.cols;
  int height = input.rows;
  // Max radius
  int rmax = (int) round(width*0.2);
  // Image size
  float da = 1.0*width/a_bin;
  float db = 1.0*height/b_bin;
  float dr = 1.0*rmax/r_bin;

  // Hough accumulator
  // Create accumulator with extra rows and cols
  int*** acc = new int**[a_bin+2];
  int i, j, k;
  for( i = 0; i < a_bin+2; ++i)
  {
    acc[i] = new int*[b_bin+2];
    for( j = 0; j < b_bin+2; ++j)
    {
      acc[i][j] = new int[r_bin+2];
      for ( k = 0; k < r_bin+2; ++k)
      {
        acc[i][j][k] = 0;  
      }
    }
  }
  
  // Hough Transform
  for( j = 0; j < height; ++j)
  {
    for ( i = 0; i < width; ++i)
    {
      // Check if the point is an edge
      if (input.at<unsigned char>(j,i) > 0)
      {
        float a = 0.0, b = 0.0;
        for (int a_i = 0; a_i < a_bin; ++a_i, a+=da)
        {
          b = 0.0;
          for (int b_i = 0; b_i < b_bin; ++b_i, b+=db)
          {
            int r_i = (int) round(sqrt((i-a)*(i-a)+(j-b)*(j-b))/dr);
            r_i = min(max(r_i, 1), r_bin);
            ++acc[a_i][b_i][r_i];
          }
        }

      }
    }
  }

  // Peaks
  vector<HoughCircle> peaks;
  const int votes_threshold = 15;
  const int neigh_threshold = 1;
  for (i = 1; i < a_bin; ++i)
  {
    for (j = 1; j < b_bin; ++j)
    {
      for (int k = 0; k < r_bin; ++k)
      {
        int votes = acc[i][j][k];
        if ( votes > votes_threshold 
          /*
          && 
          // Compare with neighborhood
          votes > neigh_threshold+acc[i-1][j-1][k-1] &&
          votes > neigh_threshold+acc[i-1][j-1][k] &&
          votes > neigh_threshold+acc[i-1][j-1][k+1] &&

          votes > neigh_threshold+acc[i-1][j][k-1] &&
          votes > neigh_threshold+acc[i-1][j][k] &&
          votes > neigh_threshold+acc[i-1][j][k+1] &&

          votes > neigh_threshold+acc[i-1][j+1][k-1] &&
          votes > neigh_threshold+acc[i-1][j+1][k] &&
          votes > neigh_threshold+acc[i-1][j+1][k+1] &&

          votes > neigh_threshold+acc[i][j-1][k-1] &&
          votes > neigh_threshold+acc[i][j-1][k] &&
          votes > neigh_threshold+acc[i][j-1][k+1] &&

          votes > neigh_threshold+acc[i][j][k-1] &&
          votes > neigh_threshold+acc[i][j][k+1] &&

          votes > neigh_threshold+acc[i][j+1][k-1] &&
          votes > neigh_threshold+acc[i][j+1][k] &&
          votes > neigh_threshold+acc[i][j+1][k+1] &&

          votes > neigh_threshold+acc[i+1][j-1][k-1] &&
          votes > neigh_threshold+acc[i+1][j-1][k] &&
          votes > neigh_threshold+acc[i+1][j-1][k+1] &&

          votes > neigh_threshold+acc[i+1][j][k-1] &&
          votes > neigh_threshold+acc[i+1][j][k] &&
          votes > neigh_threshold+acc[i+1][j][k+1] &&

          votes > neigh_threshold+acc[i+1][j+1][k-1] &&
          votes > neigh_threshold+acc[i+1][j+1][k] &&
          votes > neigh_threshold+acc[i+1][j+1][k+1]
          */
          )
        {
          HoughCircle c;
          c.acc = votes;
          c.a = i*da;
          c.b = j*db;
          c.r = k*dr;
          peaks.push_back(c);
        }
      }
    }
  }
  STREAM("Peaks: " << peaks.size());
  STREAM(" with th.: " << votes_threshold << endl);

  // Sort using votes
  HoughCircleComparator comparator;
  sort(peaks.begin(), peaks.end(), comparator);
  // Add to circles
  num_circles = min(static_cast<size_t>(num_circles), peaks.size());
  for (i = 0; i < num_circles; ++i)
  {
    circles.push_back(peaks[i]);
  }

  for (i = 1; i < a_bin; ++i)
  {
    for (j = 1; j < b_bin; ++j)
    {
      delete[] acc[i][j];
    }
    delete[] acc[i];
  }
  delete acc;
  STREAM("Result: " << num_circles);
}

/* -------- Draw lines -------- */
void drawLines(const Mat &input, Mat &output, const vector<HoughLine> &lines)
{
  // Copy image
  output = input.clone();
  
  int distance = 2*input.cols + 2*input.rows;

  // Draw images
  for (vector<HoughLine>::const_iterator l = lines.begin(); l != lines.end(); ++l)
  {
    float cost = cos(l->theta), sint = sin(l->theta);
    float x0 = cost*l->r, y0 = sint*l->r;
    // Use dista points on the line
    Point2f p0(x0-distance*sint, y0+distance*cost);
    Point2f p1(x0+distance*sint, y0-distance*cost);
    line(output, p0, p1, CV_RGB(250,0,0), 2, CV_AA);
  }
}
/* -------- Draw Circles -------- */
void drawCircles(const Mat &input, Mat &output, const vector<HoughCircle> &circles)
{
  // Copy image
  output = input.clone();
  
  // Draw images
  for (vector<HoughCircle>::const_iterator c = circles.begin(); c != circles.end(); ++c)
  {
    Point2f p(c->a, c->b);
    circle(output, p, c->r, CV_RGB(0,0,255), 2, CV_AA);
  }
}


void t2( int, void* );

Mat input, output, houghImage, src, withLines, final;
vector<HoughLine> lines;
vector<HoughCircle> circles;
string windowName = "t2";

/* -------- Parameters -------- */

// Edge threshold ET
string ETtrackbar = "Edge thres.";
int ETvalue = 160; // Binary threshold for edge detection
int const ETmax = 255;

// Number of lines NL
string NLtrackbar = "Line num.";
int NLvalue = 10;
int const NLmax = 50;

// Number of circles NC
string NCtrackbar = "Circle num.";
int NCvalue = 5;
int const NCmax = 20;

int lRBin = 100;
int lTBin = 180;

int cRBin = 50;
int cABin = 50;
int cBBin = 50;

/* -------- Main -------- */
void printHelp(const char* name)
{
    cout << "Uso: "<< name << " lRBin lTBin cRBin cABin cBBin imagen.jpg" << endl;
    cout << "Paramatros " << endl;
    cout << "\tlRBin - Num bin para radio de recta" <<endl;
    cout << "\tlTBin - Num bin para angulo de recta" <<endl;
    cout << "\tcRBin - Num bin para radio de circulo" <<endl;
    cout << "\tcABin - Num bin para posicion a del circulo" <<endl;
    cout << "\tcBBin - Num bin para posicion b del circulo" <<endl;
    cout << "Ejemplo: $ "<< name <<"100 180 50 50 50 img01.jpg" <<endl;
}


int main( int argc, char** argv)
{  
  if (argc != 7)
  {
    printHelp(argv[0]);
    exit(1);
  }

  try {
    lRBin = atoi(argv[1]);
    lTBin = atoi(argv[2]);
    cRBin = atoi(argv[3]);
    cABin = atoi(argv[4]);
    cBBin = atoi(argv[5]);
  } catch(exception &e) {
    printHelp(argv[0]);
    exit(1);
  }

  // Load an image
  src = imread( argv[6] );
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
  createTrackbar( NLtrackbar, windowName, &NLvalue, NLmax, t2);
  createTrackbar( NCtrackbar, windowName, &NCvalue, NCmax, t2);

  t2(0,0);
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

void t2( int, void* )
{
  // Apply edge detector
  float edgeThreshold = ETvalue*5.0/ETmax;
  lines.clear();
  circles.clear();

  edgeDetector(input, output, edgeThreshold);
  houghLines(output, houghImage, lines, NLvalue, lTBin, lRBin);
  houghCircles(output, houghImage, circles, NCvalue, cABin, cBBin, cRBin);
  drawLines(src, withLines, lines);
  drawCircles(withLines, final, circles);
  // Show image
  imshow( windowName, final);
}
