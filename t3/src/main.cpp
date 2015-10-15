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

void genTransform(DMatch match, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2, double &e, double &theta, double &tx, double &ty)
{
    e = 1; // Reemplazar
    theta = 0; // Reemplazar
    tx = keypoints2[match.queryIdx].pt.x - keypoints1[match.trainIdx].pt.x; // Completar para rotacion y escala
    ty = keypoints2[match.queryIdx].pt.y - keypoints1[match.trainIdx].pt.y; // Completar para rotacion y escala
}

int computeConsensus(vector<DMatch> &matches, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2, vector<int> &selected, double e, double theta, double tx, double ty)
{
    int cons = 0;
    selected.clear();
    for (int i=0; i<(int)matches.size(); i++)
    {
    	double x1 = keypoints1[matches[i].trainIdx].pt.x;
    	double y1 = keypoints1[matches[i].trainIdx].pt.y;
    	double x2 = keypoints2[matches[i].queryIdx].pt.x;
    	double y2 = keypoints2[matches[i].queryIdx].pt.y;
    	double x2t = 0; // Calcular posicion proyectada
    	double y2t = 0; // Calcular posicion proyectada
    	double ex = 0; // Calcular error de proyeccion
    	if (ex < 40)
    	{
    		selected.push_back(i);
    		cons++;
    	}
    }
    return cons;
}

bool ransac(vector<DMatch> &matches, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2, vector<DMatch> &accepted)
{
	vector<int> selected;
	double e, theta, tx, ty;
	// Completar para N intentos
	int ind = rand() % matches.size();
	genTransform(matches[ind], keypoints1, keypoints2, e, theta, tx, ty);
	int consensus = computeConsensus(matches, keypoints1, keypoints2, selected, e, theta, tx, ty);
	if (consensus > 30)
	{
		for (int i=0; i<(int)selected.size(); i++)
			accepted.push_back(matches[selected[i]]);
		return true;
	}
	return false;
}

int main(void)
{
	Mat input1, input2; // Crear matriz de OpenCV
	input1 = imread("ice1.jpg"); //Leer imagen
	input2 = imread("ice2.jpg"); //Leer imagen

	if(input1.empty() || input2.empty()) // No encontro la imagen
	{
		cout<<"Imagen no encontrada"<<endl;
		return 1; // Sale del programa anormalmente
	}

	SurfFeatureDetector detector;
	vector<KeyPoint> keypoints1;
	detector.detect(input1, keypoints1);

	vector<KeyPoint> keypoints2;
	detector.detect(input2, keypoints2);

	SurfDescriptorExtractor extractor;
	Mat descriptors1, descriptors2;
	extractor.compute(input1, keypoints1, descriptors1);
	extractor.compute(input2, keypoints2, descriptors2);

	BFMatcher matcher(NORM_L2);
	vector<DMatch> matches;
	matcher.match(descriptors2, descriptors1, matches);

	vector<DMatch> accepted;
	ransac(matches, keypoints1, keypoints2, accepted);

	// drawing the results
	Mat output;
	drawKeypoints(input1, keypoints1, output);
	imshow("keypoints1", output);
	drawKeypoints(input2, keypoints2, output);
	imshow("keypoints2", output);

	imwrite("sift_result.jpg", output);
	namedWindow("matches", 1);
	Mat img_matches;
	drawMatches(input2, keypoints2, input1, keypoints1, matches, img_matches);
	imshow("matches", img_matches);
	imwrite("matches.jpg", img_matches);

	Mat img_accepted;
	drawMatches(input2, keypoints2, input1, keypoints1, accepted, img_matches);
	imshow("accepted", img_matches);
	imwrite("accepted.jpg", img_matches);

	cout << "Presione ENTER en una ventana o CTRL-C para salir" << endl;
	waitKey(0);

	return 0;
}
