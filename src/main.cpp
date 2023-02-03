#include <iostream>
#include <opencv2/opencv.hpp>

void OurFiler(const cv::Mat& input, cv::Mat& output) {

	const auto width = input.cols;
	const auto height = input.rows;

	const int window_size = 5;

	// TEMPORARY CODE
	for (int r = 0; r < height; ++r) {
		for (int c = 0; c < width; ++c) {
			output.at<uchar>(r, c) = 0;
		}
	}

	for (int r = window_size / 2; r < height - window_size / 2; ++r) {
		for (int c = window_size / 2; c < width - window_size / 2; ++c) {

			// box filter
			int sum = 0;
			for (int i = -window_size / 2; i <= window_size / 2; ++i) {
				for (int j = -window_size / 2; j <= window_size / 2; ++j) {
					sum += input.at<uchar>(r + i, c + j);
				}
			}
			output.at<uchar>(r, c) = sum / (window_size * window_size);

			// TODO: implement Gaussian filter

		}
	}
}

int main() {

	cv::Mat im = cv::imread("lena.png", 0);

	if (im.data == nullptr) {
		std::cerr << "Failed to load image" << std::endl;
	}

	//cv::imshow("im", im);
	//cv::waitKey();

	cv::Mat noise(im.size(), im.type());
	uchar mean = 0;
	uchar stddev = 25;
	cv::randn(noise, mean, stddev);


	im += noise;

	cv::imshow("im", im);
	//cv::waitKey();

	// gaussian
	cv::Mat output;
	cv::GaussianBlur(im, output, cv::Size(7, 7), 0, 0);
	cv::imshow("gaussian", output);
	//cv::waitKey();

	// median
	cv::medianBlur(im, output, 3);
	cv::imshow("median", output);
	//cv::waitKey();

	// bilateral
	double window_size = 11;
	cv::bilateralFilter(im, output, window_size, 2 * window_size, window_size / 2);
	cv::imshow("bilateral", output);

	OurFiler(im, output);
	cv::imshow("OurFiler", output);


	cv::waitKey();

	// HW (1 point for each metric, max 5 points):
	// - compare your images
	//		- SSD
	//		- RMSE (Root Mean Squared Error)
	//		- PSNR ..
	//	  - ....

	return 0;
}