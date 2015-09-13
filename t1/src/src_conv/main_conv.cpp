#define _DEBUG
#include <iostream>
#include <cv.h>
#include <highgui.h>

using namespace std;
using namespace cv;

void convolucion(Mat input, Mat mask, Mat output)
{
	// POR HACER: Programar la convolucion aca
	for (int r=0; r<input.rows ; r++)
	{
		for (int c=0 ; c<input.cols; c++)
		{
			output.at<float>(r,c) = 255-input.at<float>(r,c);
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
	
	Mat input;
	original.convertTo(input, CV_32FC1);
	
	float maskval[9] = {0.33, 0.33, 0.33, 0, 0, 0, -0.33, -0.33, -0.33};  // horizontal
	Mat mask = Mat(3, 3, CV_32FC1, maskval);

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
