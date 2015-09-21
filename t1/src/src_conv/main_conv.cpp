#define _DEBUG
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>

//#define CONV_TEST

using namespace std;
using namespace cv;

void printMat(const Mat &mat, const string &name = "M")
{
    cout << name << " = " << endl << " "  << mat << endl << endl;
}

void convolucion(Mat input, Mat mask, Mat output)
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



int main(void)
{
    Mat originalRGB = imread("cuadrado.png"); // Read image

    if(originalRGB.empty()) // Image not found
    {
        cout << "Imagen no encontrada" << endl;
        return 1;
    }
    
    Mat original;
    cvtColor(originalRGB, original, CV_BGR2GRAY);
    
    Mat input;
    original.convertTo(input, CV_32FC1);

    //float maskval[9] = {0.33, 0.33, 0.33, 0, 0, 0, -0.33, -0.33, -0.33};  // horizontal
    Mat mask(3, 3, CV_32FC1,  Scalar(0.11));
    printMat(mask, "Mask");
    Mat output = Mat::zeros(input.rows, input.cols, CV_32FC1);  
    convolucion(input, mask, output);
    output = abs(output);
    Mat last;
    output.convertTo(last, CV_8UC1);

    imshow("filtered", last);   // Mostrar imagen
    imwrite("filtered.jpg", last); // Grabar imagen
    cvWaitKey(0); // Pausa, permite procesamiento interno de OpenCV

    return 0; // Sale del programa normalmente
}
