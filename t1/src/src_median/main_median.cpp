#define _DEBUG
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <algorithm>

using namespace std;
using namespace cv;

void mediana(Mat input, Mat output)
{
	// POR HACER: programar el filtro de mediana 3x3 aca
	vector<float> vals;
	vals.push_back(1);
	vals.push_back(2);
	vals.push_back(3);
	vals.push_back(4);
	vals.push_back(4);
	sort(&vals[0], &vals[5]);
	cout << "La mediana es " << vals[2] << endl;
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
	mediana(input, output);

	Mat last;
	output.convertTo(last, CV_8UC1);

	imshow("median", last);   // Mostrar imagen
	imwrite("median.jpg", last); // Grabar imagen
	cvWaitKey(0); // Pausa, permite procesamiento interno de OpenCV

	return 0; // Sale del programa normalmente
}
