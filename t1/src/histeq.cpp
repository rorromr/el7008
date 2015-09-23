#define _DEBUG
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <math.h>

using namespace std;
using namespace cv;

void equalization(const Mat &input, Mat &output)
{
    // Create new histogram, init on zero
    float hist[256];
    const float d = 1.0/(input.rows*input.cols);
    for (int i=0; i<256; ++i)
        hist[i] = 0.0;
    // Calc histogram
    for (int r=0; r<input.rows; ++r)
    {
        for (int c=0; c<input.cols; ++c)
        {
            hist[input.at<unsigned char>(r,c)] += d;
        }
    }
    // Look up table
    float lookup[256];
    float sum = 0.0;
    for (unsigned int i = 0; i < 256; ++i)
    {
        sum += hist[i];
        lookup[i] = sum*255 + 0.5;
    }
    // Apply transform
    for (int r=0; r<output.rows; ++r)
    {
        for (int c=0; c<output.cols; ++c)
        {
            output.at<unsigned char>(r,c) = (unsigned char) lookup[input.at<unsigned char>(r,c)];
        }
    }
}

int main(int argc, char** argv)
{
    if( argc != 2)
    {
        cout << "Uso: "<< argv[0] << " imagen.png" << endl;
        return 1;
    }

    string image_name(argv[1]);
    Mat originalRGB = imread(image_name); //Leer imagen

    if(originalRGB.empty()) // No encontro la imagen
    {
        cout << "Imagen no encontrada" << endl;
        return 1;
    }
    
    Mat original;
    cvtColor(originalRGB, original, CV_BGR2GRAY);
    
    Mat output = Mat::zeros(original.rows, original.cols, CV_8UC1);
    equalization(original, output);

    imshow("Filter: " + image_name, output);   // Show
    
    string filtered_name=image_name.substr(0,image_name.find_last_of('.'))+"_eq.jpg";
    imwrite(filtered_name, output); // Save
    
    waitKey(1000);

    return 0; // Sale del programa normalmente
}
