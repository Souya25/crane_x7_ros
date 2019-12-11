#include <iostream>
#include <opencv2/opencv.hpp>

int main(){

	cv::Mat gray, sobel, laplacian, canny; //画像格納用のオブジェクト宣言
	
	gray = cv::imread("desk.jpg", 0); //画像データの読み込み
	
	if (gray.empty()) { //エラー処理
		std::cout << "error" << std::endl;
		return 0;
	}
	
	Sobel(gray, sobel, CV_32F, 1, 1, 1, 5); //Sobel法 //入力画像,出力画像,ビット深度,xの微分次数,yの微分次数,カーネルのサイズ,結果に加える値
	
	Laplacian(gray, laplacian, CV_32F, 1, 5); //Laplacian法 //入力画像,出力画像,ビット深度,アパーチャサイズ,結果に加える値
	
	Canny(gray,canny, 150, 250); //Cannuy法 //入力画像,出力画像,閾値1,閾値2
	
	//画像の保存
	cv::imwrite("sobel.jpg", sobel);	
	cv::imwrite("laplacian.jpg", laplacian);
	cv::imwrite("canny.jpg", canny);

	sobel.release();
	laplacian.release();
	canny.release();

	return 0;
}
