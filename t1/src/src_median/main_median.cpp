#define _DEBUG
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <algorithm>

using namespace std;
using namespace cv;

void mediana(const Mat &input, Mat &output, unsigned int window_size = 3)
{
    // Central position
    const int mask_anchor = window_size/2;

    for (int r=0; r<input.rows ; ++r)
    {
        for (int c=0 ; c<input.cols; ++c)
        {
            // Create vector
            std::vector<float> elements;
            elements.reserve(window_size*window_size);
            // Add zero and matrix elements to padding matrix
            for (int pad_r=0; pad_r<window_size; ++pad_r)
            {
                for (int pad_c=0; pad_c<window_size; ++pad_c)
                {
                    int offset_r = r - mask_anchor + pad_r;
                    int offset_c = c - mask_anchor + pad_c;
                    
                    elements.push_back(
                        (offset_r>=0 && offset_r < input.rows &&
                         offset_c>=0 && offset_c < input.cols) ? input.at<float>(offset_r,offset_c) : 127);
                }
            }
            // Sort elements
            sort(elements.begin(), elements.end());
            // Choose middle position element
            output.at<float>(r,c) = elements[window_size*window_size/2];
        }
    }
}

int main(void)
{
    Mat originalRGB = imread("saltpepper.jpg"); //Leer imagen

    if(originalRGB.empty()) // No encontro la imagen
    {
        cout << "Imagen no encontrada" << endl;
        return 1;
    }
    
    Mat original;
    cvtColor(originalRGB, original, CV_BGR2GRAY);
    
    Mat input;
    original.convertTo(input, CV_32FC1);
    
    Mat output = Mat::zeros(input.rows, input.cols, CV_32FC1);  
    mediana(input, output, 5);

    Mat last;
    output.convertTo(last, CV_8UC1);

    imshow("median", last);   // Mostrar imagen
    imwrite("median.jpg", last); // Grabar imagen
    cvWaitKey(0); // Pausa, permite procesamiento interno de OpenCV

    return 0; // Sale del programa normalmente
}
