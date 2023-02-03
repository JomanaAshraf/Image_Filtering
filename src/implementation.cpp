#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
using namespace std::chrono;

void CreateGaussianMask(const int& window_size,const double& sigma, cv::Mat& mask, int& sum_mask)
{
	cv::Size mask_size(window_size, window_size);
	mask = cv::Mat(mask_size, CV_8UC1);

	const double hw = window_size / 2;
	const double sigmaSq = sigma * sigma;

	for (int r = 0; r < window_size; ++r) {
		for (int c = 0; c < window_size; ++c) {
			//mask.at<uchar>(r, c) = 1; // box filter

			// TODO: implement Gaussian filter
			double r2 = (r - hw) * (r - hw) + (c - hw) * (c - hw); // distance squared from center of the mask
			mask.at<uchar>(r, c) = 255 * std::exp(-r2 / (2 * sigmaSq));
			// 0..1 -> 0..255
		}
	}


	for (int r = 0; r < window_size; ++r) {
		for (int c = 0; c < window_size; ++c) {
			sum_mask += static_cast<int>(mask.at<uchar>(r, c));
		}
	}
}

void OurBilateralFilter(const cv::Mat& input,const int& window_size ,const double& sigma,const float& sigmaRange, cv::Mat& output) {

	const auto width = input.cols;
	const auto height = input.rows;

	// TEMPORARY CODE
	for (int r = 0; r < height; ++r) {
		for (int c = 0; c < width; ++c) {
			output.at<uchar>(r, c) = 0;
		}
	}

	cv::Mat mask;
	int sum_mask = 0; // in order to normalize filtering

	CreateGaussianMask(window_size, sigma, mask, sum_mask);

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


void JointBilateralFilter(const cv::Mat& input,const cv::Mat& input2,const int& window_size ,const double& sigma,const float& sigmaRange, cv::Mat& output) {

	const auto width = input.cols;
	const auto height = input.rows;

	// TEMPORARY CODE
	for (int r = 0; r < height; ++r) {
		for (int c = 0; c < width; ++c) {
			output.at<uchar>(r, c) = 0;
		}
	}

	cv::Mat mask;
	int sum_mask = 0; // in order to normalize filtering

	CreateGaussianMask(window_size,sigma,mask, sum_mask);

	// const float sigmaRange = 20; // TODO: experiment
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


void JointBilateralUpsampling(const cv::Mat& input,const cv::Mat& input2,const int& window_size ,const double& sigma,const float& sigmaRange, cv::Mat& output) {

	const auto width = input.cols;
	const auto height = input.rows;

	const auto width_depth = input2.cols;
	const auto height_depth = input2.rows;

	

	// TEMPORARY CODE
	for (int r = 0; r < height; ++r) {
		for (int c = 0; c < width; ++c) {
			output.at<uchar>(r, c) = 0;
		}
	}

	cv::Mat mask;
	int sum_mask = 0; // in order to normalize filtering

	CreateGaussianMask(window_size,sigma, mask, sum_mask);

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
			float sum_jointBilateralup_mask = 0;

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

					int rlow=round(((r+i)/static_cast<float>(height))*height_depth);
					int clow=round(((c+j)/static_cast<float>(width))*width_depth);

					sum += static_cast<int>(input2.at<uchar>(rlow,clow)) * weight; // convolution happening...
					sum_jointBilateralup_mask += weight;
				}
			}
			output.at<uchar>(r, c) = sum / sum_jointBilateralup_mask; // normalization
		}
	}
}


void IterativeUpsampling(const cv::Mat& input,const cv::Mat& input2,const int& window_size ,const double& sigma,const float& sigmaRange, cv::Mat& output){

	int uf = log2(input.rows / input2.rows); // upsample factor
	cv::Mat low;
	input2.copyTo(low); // lowres depth image
	cv::Mat high;
	input.copyTo(high); // highres rgb image

	for (int i = 1; i <= (uf-1); ++i)
	{
		// cv::Mat low_high;

		cv::resize(low, low, low.size() * 2); // doubling the size of the depth image
		cv::resize(input, high, low.size());		// resizing the rgb image to depth image size

		JointBilateralFilter(high, low,window_size,sigma,sigmaRange, low); // applying the joint bilateral filter with changed size depth and rbg images
	}
	cv::resize(low, low, input.size()); // in the end resizing the depth image to rgb image size
	JointBilateralFilter(input,low,window_size,sigma,sigmaRange,output);// applying the joint bilateral filter with full res. size images
}



void Disparity2PointCloud(
  const std::string& output_file,const std::string& filter,cv::Mat& disparities,
  const int& window_size,const int& dmin, const double& baseline, const double& focal_length)
{
	std::stringstream out3d;
	out3d << output_file << "_"<< filter<<".xyz";
	std::ofstream outfile(out3d.str());

	int height = disparities.size().height;
	int width = disparities.size().width;

	for (int i = 0; i < height - window_size; ++i) {
		std::cout << "Reconstructing 3D point cloud from disparities... " << std::ceil(((i) / static_cast<double>(height - window_size + 1)) * 100) << "%\r" << std::flush;
		for (int j = 0; j < width - window_size; ++j) {
		if (disparities.at<uchar>(i, j) == 0) continue;

		const double u1 = j - width/2.;    
		const double v1 = i - height/2.;
		const double v2 = v1;
		double d = static_cast<double>(disparities.at<uchar>(i,j))+ dmin;
		const double u2 = u1+d;

		const double Z = (baseline*focal_length)/(d);
		const double X = -((baseline*(u1+u2))/(2*d));
		const double Y = (baseline*(v1))/d;

		outfile << X << " " << Y << " " << Z << std::endl;
		}
	}
	std::cout << "Reconstructing 3D point cloud from disparities... Done.\r" << std::flush;
	std::cout << std::endl;
}

int main(int argc, char** argv) {
	char c;
	if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " IMAGE1 IMAGE2 OUTPUT_FILE" << std::endl;
    return 1;
  }
	cv::Mat im = cv::imread(argv[1], 0);
	cv::Mat im2 = cv::imread(argv[2], 0);
	// const std::string output_file = argv[3];

	if (im.data == nullptr) {
		std::cerr << "Failed to load image" << std::endl;
	}

	// camera setup parameters
	const double focal_length = 3740;
	const double baseline = 160;

	// stereo estimation parameters
	const int dmin = 200;

	cv::Mat noise(im.size(), im.type());
	uchar mean = 0;
	uchar stddev = 25; // standard deviation
	cv::randn(noise, mean, stddev);

	im += noise; // "input"

	cv::imshow("im", im);
	
	cv::Mat output;
	cv::Mat downsampled_im;

	cv::resize(im2, downsampled_im, im2.size()/5);

	char line[256];
  	FILE *fp;
	c=1;
	while(c == 1){
			fp=popen("zenity --forms \
				--title='Upsampling' \
				--text='Upsampling Parameters' \
				--separator=',' \
				--add-entry='window_size' \
				--add-entry='sigma_range' \
				--add-entry='output_file_name' \
				","r");
			if (fp==NULL) {
				perror("Pipe returned a error");
			} else {
			
				fgets(line,sizeof(line),fp);
				//Define string data that will be splitted
				std::string strData = line;

				//Define contant data that will be worked as delimiter
				const char separator = ',';

				//Define the dynamic array variable of strings
				std::vector<std::string> outputArray;

				//Construct a stream from the string
				std::stringstream streamData(strData);

				std::string val;

				while (std::getline(streamData, val, separator)) {
					outputArray.push_back (val);
				}
				int window_size=stoi(outputArray[0]);
				const float sigmaRange=std::stof(outputArray[1]);
				const std::string output_file=outputArray[2];
				std::cout<<"Window_size = "<<window_size<<std::endl;
				std::cout<<"sigma_spectral = "<<sigmaRange<<std::endl;
				std::cout<<"Output_file_name = "<<output_file<<std::endl;
				
			
				const double hw = window_size / 2;
				const double sigma = std::sqrt(2.0) * hw / 2.5;
				cv::bilateralFilter(im, output, window_size, 2 * window_size, window_size / 2);

			//  Joint Bilateral Upsampling

				// Get the time before calling the function
				auto start_JBU = high_resolution_clock::now();
				JointBilateralUpsampling(im,downsampled_im,window_size,sigma,sigmaRange,output);
				cv::imshow("JointBilateralUpsampling", output);
				// Get time after the function is implemented
				auto stop_JBU = high_resolution_clock::now();

				std::stringstream out1;
				out1 << output_file << "_JBU.png";
				cv::imwrite(out1.str(), output);
				
				// Calculate the time taken in minutes and seconds
				auto duration_JBU = duration_cast<seconds>(stop_JBU - start_JBU);
				int seconds_JBU = duration_JBU.count();
				int minutes_JBU = seconds_JBU / 60;
				std::cout << "Time_JBU taken = " << minutes_JBU<< "m" <<" "<< int(seconds_JBU%60) << "s"<<std::endl;

			// Iterative Upsampling

				cv::Mat output2;
				cv::bilateralFilter(im, output2, window_size, 2 * window_size, window_size / 2);

				// Get the time before calling the function
				auto start_IU = high_resolution_clock::now();
				IterativeUpsampling(im,downsampled_im,window_size,sigma,sigmaRange,output2);
				cv::imshow("IterativeUpsampling", output2);
				// Get time after the function is implemented
				auto stop_IU = high_resolution_clock::now();

				std::stringstream out2;
				out2 << output_file << "_IU.png";
				cv::imwrite(out2.str(), output2);
				
				// Calculate the time taken in minutes and seconds
				auto duration_IU = duration_cast<seconds>(stop_IU - start_IU);
				int seconds_IU = duration_IU.count();
				int minutes_IU = seconds_IU / 60;
				std::cout << "Time_IU taken = " << minutes_IU<< "m" <<" "<< int(seconds_IU%60) << "s"<<std::endl;



			/// Saving the time for both algorithms

				// saving the time taken to get the results in a text file
				std::ofstream txt_file;
				// File Open
				txt_file.open(output_file +".txt");

				// Write to the file
				txt_file << "Window_size = "<< window_size << std::endl;
				txt_file << "sigma = "<< sigma << std::endl;
				txt_file << "sigmaRange = "<< sigmaRange << std::endl;
				txt_file << "Time_JBU = "<< seconds_JBU << std::endl;
				txt_file << "Time_IU = "<< seconds_IU << std::endl;


				// File Close
				txt_file.close();

				const std::string str1="JBU";
				const std::string str2="IU";

				// Disparity2PointCloud(output_file,str1,output,window_size, dmin, baseline, focal_length);
				// Disparity2PointCloud(output_file,str2,output2,window_size, dmin, baseline, focal_length);
				cv::waitKey();
				if (c!=27){
					cv::destroyAllWindows();	
					c=1;
				}
			
		}
		
	}	
	return 0;
}

