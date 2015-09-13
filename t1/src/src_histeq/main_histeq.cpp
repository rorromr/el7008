#define _DEBUG
#include <iostream>
#include <cv.h>
#include <highgui.h>

using namespace std;
using namespace cv;

void ecualizar(Mat input, Mat output)
{
	// POR HACER: programar la ecualizacion de histograma aca
	float hist[256];
	for (int i=0; i<256; i++)
		hist[i] = 0;
	for (int r=0; r<input.rows; r++)
	{
		for (int c=0; c<input.cols; c++)
		{
			int ind = input.at<unsigned char>(r,c);
			hist[ind] = hist[ind]+1;
		}
	}
	return;
}

int main(void)
{
	Mat originalRGB = imread("mala_ilum.jpg"); //Leer imagen

	if(originalRGB.empty()) // No encontro la imagen
	{
		cout << "Imagen no encontrada" << endl;
		return 1;
	}
	
	Mat original;
	cvtColor(originalRGB, original, CV_BGR2GRAY);
	
	Mat output = Mat::zeros(original.rows, original.cols, CV_8UC1);
	ecualizar(original, output);

	imshow("ecualizado", output);   // Mostrar imagen
	imwrite("ecualizado.jpg", output); // Grabar imagen
	cvWaitKey(0); // Pausa, permite procesamiento interno de OpenCV

	return 0; // Sale del programa normalmente
}
