#include <iostream>
#include <opencv2/opencv.hpp>
// #include "main.h"

void CreateGaussianMask(const int& window_size, cv::Mat& mask, int& sum_mask)
{
	cv::Size mask_size(window_size, window_size);
	mask = cv::Mat(mask_size, CV_8UC1);

	const double hw = window_size / 2;
	const double sigma = std::sqrt(2.0) * hw / 2.5; // ? maybe half_window_size / 2.5
	const double sigmaSq = sigma * sigma;

	// rmax = 2.5 * sigma
	// sigma = rmax / 2.5

	for (int r = 0; r < window_size; ++r) {
		for (int c = 0; c < window_size; ++c) {
			//mask.at<uchar>(r, c) = 1; // box filter

			// TODO: implement Gaussian filter
			double r2 = (r - hw) * (r - hw) + (c - hw) * (c - hw); // distance squared from center of the mask
			mask.at<uchar>(r, c) = 255 * std::exp(-r2 / (2 * sigmaSq));
			std::cout << static_cast<int>(mask.at<uchar>(r, c)) << std::endl;
			// 0..1 -> 0..255
		}
		std::cout << std::endl;
	}

	std::cout << mask << std::endl;

	for (int r = 0; r < window_size; ++r) {
		for (int c = 0; c < window_size; ++c) {
			sum_mask += static_cast<int>(mask.at<uchar>(r, c));
		}
	}
}

void OurFiler(const cv::Mat& input, cv::Mat& output) {

	const auto width = input.cols;
	const auto height = input.rows;

	const int window_size = 17;

	// TEMPORARY CODE
	for (int r = 0; r < height; ++r) {
		for (int c = 0; c < width; ++c) {
			output.at<uchar>(r, c) = 0;
		}
	}

	cv::Mat mask;
	int sum_mask = 0; // in order to normalize filtering

	CreateGaussianMask(window_size, mask, sum_mask);

	for (int r = window_size / 2; r < height - window_size / 2; ++r) {
		for (int c = window_size / 2; c < width - window_size / 2; ++c) {

			int sum = 0;
			for (int i = -window_size / 2; i <= window_size / 2; ++i) {
				for (int j = -window_size / 2; j <= window_size / 2; ++j) {
					int intensity = static_cast<int>(input.at<uchar>(
						r + i,
						c + j));
					int weight = static_cast<int>(mask.at<uchar>(
						i + window_size / 2,
						j + window_size / 2));

					sum += intensity * weight; // convolution happening...
				}
			}
			output.at<uchar>(r, c) = sum / sum_mask; // normalization
		}
	}
}

void OurBilateralFilter(const cv::Mat& input, cv::Mat& output) {

	const auto width = input.cols;
	const auto height = input.rows;

	const int window_size = 17;

	// TEMPORARY CODE
	for (int r = 0; r < height; ++r) {
		for (int c = 0; c < width; ++c) {
			output.at<uchar>(r, c) = 0;
		}
	}

	cv::Mat mask;
	int sum_mask = 0; // in order to normalize filtering

	CreateGaussianMask(window_size, mask, sum_mask);

	const float sigmaRange = 20; // TODO: experiment
	const float sigmaRangeSq = sigmaRange * sigmaRange;

	float range_mask[256];
	// compute range kernel
	for (int diff = 0; diff < 256; ++diff) {
		range_mask[diff] = std::exp(-diff * diff / (2 * sigmaRangeSq));
	}

	for (int r = window_size / 2; r < height - window_size / 2; ++r) {
		for (int c = window_size / 2; c < width - window_size / 2; ++c) {
			// get center intensity
			int intensity_center = static_cast<int>(input.at<uchar>(r, c));

			int sum = 0;
			float sum_Bilateral_mask = 0;

			for (int i = -window_size / 2; i <= window_size / 2; ++i) {
				for (int j = -window_size / 2; j <= window_size / 2; ++j) {
					int intensity = static_cast<int>(input.at<uchar>(
						r + i,
						c + j));
					// compute range difference to center pixel value
					int diff = std::abs(intensity_center - intensity); // 0..255
					// compute the range kernel's value
					float weight_range = range_mask[diff]; // std::exp(-diff * diff / (2 * sigmaRangeSq));

					int weight_spatial = static_cast<int>(mask.at<uchar>(
						i + window_size / 2,
						j + window_size / 2));
					// ... combine weights...
					float weight = weight_range * weight_spatial;

					sum += intensity * weight; // convolution happening...
					sum_Bilateral_mask += weight;
				}
			}
			output.at<uchar>(r, c) = sum / sum_Bilateral_mask; // normalization
		}
	}
}

void JointBilateralFilter(const cv::Mat& input,const cv::Mat& input2, cv::Mat& output) {

	const auto width = input.cols;
	const auto height = input.rows;

	const int window_size = 17;

	// TEMPORARY CODE
	for (int r = 0; r < height; ++r) {
		for (int c = 0; c < width; ++c) {
			output.at<uchar>(r, c) = 0;
		}
	}

	cv::Mat mask;
	int sum_mask = 0; // in order to normalize filtering

	CreateGaussianMask(window_size, mask, sum_mask);

	const float sigmaRange = 20; // TODO: experiment
	const float sigmaRangeSq = sigmaRange * sigmaRange;

	float range_mask[256];
	// compute range kernel
	for (int diff = 0; diff < 256; ++diff) {
		range_mask[diff] = std::exp(-diff * diff / (2 * sigmaRangeSq));
	}

	for (int r = window_size / 2; r < height - window_size / 2; ++r) {
		for (int c = window_size / 2; c < width - window_size / 2; ++c) {
			// get center intensity
			int intensity_center = static_cast<int>(input.at<uchar>(r, c));

			int sum = 0;
			float sum_jointBilateral_mask = 0;

			for (int i = -window_size / 2; i <= window_size / 2; ++i) {
				for (int j = -window_size / 2; j <= window_size / 2; ++j) {
					int intensity = static_cast<int>(input.at<uchar>(r + i,c + j));

					// compute range difference to center pixel value
					int diff = std::abs(intensity_center - intensity); // 0..255
					// compute the range kernel's value
					float weight_range = range_mask[diff]; // std::exp(-diff * diff / (2 * sigmaRangeSq));

					int weight_spatial = static_cast<int>(mask.at<uchar>(
						i + window_size / 2,
						j + window_size / 2));
					// ... combine weights...
					float weight = weight_range * weight_spatial;

					sum += static_cast<int>(input2.at<uchar>(r + i,c + j)) * weight; // convolution happening...
					sum_jointBilateral_mask += weight;
				}
			}
			output.at<uchar>(r, c) = sum / sum_jointBilateral_mask; // normalization
		}
	}
}

double NCC(const cv::Mat& image_1, const cv::Mat& image_2,double &NCC_out){
	double mean1 = 0;
	double mean2 = 0;
	double sum1=  0;
	double sum2= 0;
	double sum3=0;
	
   	int h = image_1.size().height;
	int w = image_1.size().width;
   	for (int i=0; i<h; i++){
      for (int j=0; j<w; j++){
        mean1 += image_1.at<uchar>( i, j);
		mean2+=image_2.at<uchar>( i, j);
	  }
   	}
	(int)mean1/(h*w);
	(int)mean2/(h*w);
	for (int i=0; i<h; i++){
		for (int j=0; j<w; j++){
			sum1+=(image_1.at<uchar>( i, j)-mean1)*(image_2.at<uchar>( i, j)-mean2);
			sum2+=(image_1.at<uchar>( i, j)-mean1)*(image_1.at<uchar>( i, j)-mean1);
			sum3+=(image_2.at<uchar>( i, j)-mean2)*(image_2.at<uchar>( i, j)-mean2);
		}
	}
	NCC_out=sum1/(std::sqrt(sum2*sum3));
return NCC_out;
}


int main(int argc, char** argv) {

	if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " IMAGE1 IMAGE2 OUTPUT_FILE" << std::endl;
    return 1;
  }
	// cv::Mat im = cv::imread("lena.png", 0);
	cv::Mat im = cv::imread(argv[1], 0);
	cv::Mat im2 = cv::imread(argv[2], 0);
	// const std::string output_file = argv[3];

	if (im.data == nullptr) {
		std::cerr << "Failed to load image" << std::endl;
	}

	//cv::imshow("im", im);
	//cv::waitKey();

	cv::Mat noise(im.size(), im.type());
	uchar mean = 0;
	uchar stddev = 25; // standard deviation
	cv::randn(noise, mean, stddev);

	//im.copyTo(...) // gt...

	im += noise; // "input"

	cv::imshow("im", im);
	//cv::waitKey();

	// gaussian
	cv::Mat output;
	// cv::GaussianBlur(im, output, cv::Size(7, 7), 0, 0);
	// cv::imshow("gaussian", output);
	// //cv::waitKey();

	// // median
	// cv::medianBlur(im, output, 3);
	// cv::imshow("median", output);
	// // //cv::waitKey();

	// bilateral
	double window_size = 11;
	cv::bilateralFilter(im, output, window_size, 2 * window_size, window_size / 2);
	cv::imshow("bilateral", output);

	OurFiler(im, output);
	cv::imshow("OurFiler", output);

	OurBilateralFilter(im, output);
	cv::imshow("OurBilateralFilter", output);

	JointBilateralFilter(im,im2, output);
	cv::imshow("JointBilateralFilter", output);

	cv::waitKey();



	// HW (1 point for each metric, max 5 points):
	// NCC Method
	// double NCC_out;
	// NCC(ground_truth,output,NCC_out);
	// std::cout << "The NCC value is: " << NCC_out << std::endl;
	// //SSD Method
	// int h = ground_truth.size().height;
	// int w = ground_truth.size().width;
	// float SSD=0;

	// for (int i = 0; i < h; i++){
	// 	for (int j = 0; j < w; j++) {
	// 		SSD+=pow(((static_cast<float>(ground_truth.at<uchar>( i, j))))-(static_cast<float>(output.at<uchar>(i , j ))),2);
	// 	}
	// }

	// // RMSE & PSNR Methods
	// double mse=SSD/ground_truth.total();
	// double rmse=std::sqrt(mse);
	// double psnr = 10.0*log10((255*255)/mse);

	// std::cout << "The PSNR value is: " << psnr << std::endl
    //     << "The RMSE value is : " << rmse <<std:: endl
	// 	<< "The SSD value is : " << SSD <<std:: endl;

	//cv::waitKey();
	return 0;
}