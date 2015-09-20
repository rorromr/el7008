#define _DEBUG
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>

#define CONV_TEST

using namespace std;
using namespace cv;

void printMat(const Mat &mat, const string &name = "M")
{
    cout << name << " = " << endl << " "  << mat << endl << endl;
}

void convolucion(Mat input, Mat mask, Mat output)
{
    // Matriz de zero-padding
    Mat padding = Mat::zeros(mask.rows, mask.cols, CV_32FC1);
    // Posicion central
    int mask_anchor_r = padding.rows/2, mask_anchor_c = padding.cols/2;


    for (int r=0; r<input.rows ; ++r)
    {
        for (int c=0 ; c<input.cols; ++c)
        {
            // Crear padding anadiendo ceros
            for (int pad_r=0; pad_r<padding.rows; ++pad_r)
            {
                for (int pad_c=0; pad_c<padding.cols; ++pad_c)
                {
                    int offset_r = r - mask_anchor_r + pad_r;
                    int offset_c = c - mask_anchor_c + pad_c;
                    padding.at<float>(pad_r,pad_c) = 
                        (offset_r>=0 && offset_r < input.rows &&
                         offset_c>=0 && offset_c < input.cols) ? input.at<float>(r,c) : 0;
                }
            }
            #ifdef CONV_TEST
            stringstream text;
            text << "Pad for (" << r << "," << c << ")";
            printMat(padding, text.str());
            #endif
            /*
            cout << padding.dot(mask) << endl;
            */
            output.at<float>(r,c) = padding.dot(mask);
        }
    }
}



int main(void)
{
    Mat originalRGB = imread("cuadrado.png"); //Leer imagen

    if(originalRGB.empty()) // No encontro la imagen
    {
        cout << "Imagen no encontrada" << endl;
        return 1;
    }
    
    Mat original;
    cvtColor(originalRGB, original, CV_BGR2GRAY);
    
#ifdef CONV_TEST
    cout << "Filtro de prueba" << endl;
    float input_test[9] = {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0};
    Mat input = Mat(3, 3, CV_32FC1, input_test);
    printMat(input, "Input");
#else
    Mat input;
    original.convertTo(input, CV_32FC1);
#endif
    float maskval[9] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    //float maskval[9] = {0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11};
    //float maskval[9] = {0.33, 0.33, 0.33, 0, 0, 0, -0.33, -0.33, -0.33};  // horizontal
    Mat mask = Mat(3, 3, CV_32FC1, maskval);
    printMat(mask, "Mask");
    Mat output = Mat::zeros(input.rows, input.cols, CV_32FC1);  
    convolucion(input, mask, output);
    output = abs(output);
#ifdef CONV_TEST
    printMat(output, "Out");
#endif
    Mat last;
    output.convertTo(last, CV_8UC1);

    imshow("filtered", last);   // Mostrar imagen
    imwrite("filtered.jpg", last); // Grabar imagen
    cvWaitKey(0); // Pausa, permite procesamiento interno de OpenCV

    return 0; // Sale del programa normalmente
}
