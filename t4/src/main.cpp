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
void t4( int, void* );
/* -------- Global params -------- */


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
