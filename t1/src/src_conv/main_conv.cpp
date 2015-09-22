#define _DEBUG
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>

using namespace std;
using namespace cv;

void printMat(const Mat &mat, const string &name = "M")
{
    cout << name << " = " << endl << " "  << mat << endl << endl;
}

Mat getGaussKernel(int size, float sigma)
{
    cv::Mat kernel(size,size,CV_32FC1);
    float halfSize = size / 2.0; 
    sigma = 0.3*((size-1)*0.5 - 1) + 0.8;
    sigma = 1;
    for (int i=0; i<kernel.rows;++i)
    {
        for (int j=0; j<kernel.cols;++j)
        { 
            float x = j - halfSize;
            float y = i - halfSize;
            kernel.at<float>(j,i) = 1.0/sqrt(2*M_PI*sigma*sigma)*exp(-(x*x + y*y)/(2.0*sigma*sigma));
        }
    }
    return kernel;
}

Mat getGaussLinKernel(int size, float sigma)
{
    cv::Mat kernel(size,1,CV_32FC1);
    float halfSize = size / 2.0;
    for (int i=0; i<size;++i)
    {
        float x = i - halfSize;
        kernel.at<float>(i,0) = 1.0/sqrt(2*M_PI*sigma)*exp(-(x*x)/(2.0*sigma));
    }
    return kernel;
}

void convolucion(const Mat &input, const Mat &mask, Mat &output)
{
    // Zero-padding matrix
    Mat padding = Mat::zeros(mask.rows, mask.cols, CV_32FC1);
    // Central position
    int mask_anchor_r = padding.rows/2, mask_anchor_c = padding.cols/2;

    for (int r=0; r<input.rows ; ++r)
    {
        for (int c=0 ; c<input.cols; ++c)
        {
            // Add zero and matrix elements to padding matrix
            for (int pad_r=0; pad_r<padding.rows; ++pad_r)
            {
                for (int pad_c=0; pad_c<padding.cols; ++pad_c)
                {
                    int offset_r = r - mask_anchor_r + pad_r;
                    int offset_c = c - mask_anchor_c + pad_c;
                    
                    padding.at<float>(pad_r,pad_c) = 
                        (offset_r>=0 && offset_r < input.rows &&
                         offset_c>=0 && offset_c < input.cols) ? input.at<float>(offset_r,offset_c) : 0;
                }
            }
            // Calc dot product between mask and padding
            output.at<float>(r,c) = padding.dot(mask);
        }
    }
}

void printHelp(const char* name)
{
    cout << "Uso: "<< name << " tipo_filtro imagen.png" << endl;
    cout << "tipo_filtro " << endl;
    cout << "\ta - Pasa bajos recto" <<endl;
    cout << "\tb - Pasa bajos unidimensional por filas y columnas" <<endl;
    cout << "\tc - Gaussiano" <<endl;
    cout << "\td - Gaussiano unidimensional por filas y columnas" <<endl;
    cout << "\te - Prewitt vertical" <<endl;
    cout << "\tf - Prewitt horizontal" <<endl;
    cout << "Ejemplo Prewitt horizontal: $ "<< name <<" f cuadrado.png" <<endl;
}


int main(int argc, char** argv)
{
    if( argc != 3)
    {
        printHelp(argv[0]);
        return 1;
    }

    Mat originalRGB = imread(argv[2]); // Read image

    if(originalRGB.empty()) // Image not found
    {
        cout << "Imagen no encontrada" << endl;
        return 1;
    }
    
    Mat original;
    cvtColor(originalRGB, original, CV_BGR2GRAY);
    
    Mat input;
    original.convertTo(input, CV_32FC1);

    Mat mask;
    char filtro = argv[1][0];
    switch (filtro)
    {
        case 'a':
            mask = Mat(3, 3, CV_32FC1,  Scalar(1.0/9));
            break;
        case 'b':
            mask = Mat(1, 3, CV_32FC1,  Scalar(1.0/3));
            break;
        case 'c':
            mask = getGaussKernel(5, 1.0);
            break;
        case 'd':
            mask = getGaussLinKernel(5, 1.0);
            break;
        case 'e':
            mask = (Mat_<float>(3,3) << -1, 0, 1, -1, 0, 1, -1, 0, 1)*(1.0/9);
            break;
        case 'f':
            mask = (Mat_<float>(3,3) << -1, -1, -1, 0, 0, 0, 1, 1, 1)*(1.0/9);
            break;
        default:
            cout << "\033[1;31m" << "Filtro seleccionado no existe" << "\033[0m" << endl;
            printHelp(argv[0]);
            return 1;
            break;
    }


    //printMat(mask, "Mask");
    Mat output = Mat::zeros(input.rows, input.cols, CV_32FC1);  
    convolucion(input, mask, output);
    
    // Casos filtros lineales
    if (filtro == 'b' || filtro == 'd')
    {
        Mat aux(output);
        transpose(mask, mask);
        convolucion(aux, mask, output);
    }

    output = abs(output);
    Mat last;
    output.convertTo(last, CV_8UC1);

    imshow("filtered", last);   // Mostrar imagen
    imwrite("filtered.jpg", last); // Grabar imagen
    cvWaitKey(0); // Pausa, permite procesamiento interno de OpenCV

    return 0; // Sale del programa normalmente
}
