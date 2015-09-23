#define _DEBUG
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <string.h>
#include <unistd.h>

using namespace std;
using namespace cv;

void median(const Mat &input, Mat &output, unsigned int window_size = 3)
{
    // Central position
    const int mask_anchor = window_size/2;
    std::vector<float> elements(window_size*window_size, 0.0);
    for (int r=0; r<input.rows ; ++r)
    {
        for (int c=0 ; c<input.cols; ++c)
        {            
            // Add zero and matrix elements to padding matrix
            for (int pad_r=0; pad_r<window_size; ++pad_r)
            {
                for (int pad_c=0; pad_c<window_size; ++pad_c)
                {
                    int offset_r = r - mask_anchor + pad_r;
                    int offset_c = c - mask_anchor + pad_c;
                    
                    elements[pad_r+window_size*pad_c] =
                        (offset_r>=0 && offset_r < input.rows &&
                         offset_c>=0 && offset_c < input.cols) ? input.at<float>(offset_r,offset_c) : 127;
                }
            }
            // Sort elements
            sort(elements.begin(), elements.end());
            // Choose middle position element
            output.at<float>(r,c) = elements[window_size*window_size/2];
        }
    }
}

int main(int argc, char** argv)
{
    if( argc != 2)
    {
        cout << "Uso: "<< argv[0] << " imagen.jpg" << endl;
        return 1;
    }

    string image_name(argv[1]);
    Mat originalRGB = imread(image_name); // Read img

    if(originalRGB.empty()) // Not found
    {
        cout << "Imagen no encontrada" << endl;
        return 1;
    }
    
    Mat original;
    cvtColor(originalRGB, original, CV_BGR2GRAY);
    
    Mat input;
    original.convertTo(input, CV_32FC1);
    
    Mat output = Mat::zeros(input.rows, input.cols, CV_32FC1);  
    median(input, output, 3);

    Mat last;
    output.convertTo(last, CV_8UC1);

    imshow("Filter: " + image_name, last);   // Show
    
    string filtered_name=image_name.substr(0,image_name.find_last_of('.'))+"_filtered.jpg";
    imwrite(filtered_name, last); // Save
    
    waitKey(1000);


    return 0;
}
