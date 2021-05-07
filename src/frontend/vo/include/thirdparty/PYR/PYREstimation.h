#pragma once

#include "opencv2/opencv.hpp"
//#include "Compensation.h"
#include "PhaseCorrelation.h"

#include <boost/property_tree/ptree.hpp>

using namespace std;
using namespace cv;

class PYREstimation
{
private:
	//windows info
	vector<Point2i> wind_centers;
	Point2i center_windows;
	vector<vector< int > >  windows;
	int number_windows;
	int wind_width;
	int wind_height;

	//parameters info	
	float tukey_param;
	Mat intrinsics;

	bool reuse_prev_fft_;
	float thresh_structure_;
	float thresh_distance_;

	//estimated results
	//vector<Compensation> vec_compensation;
	vector<Point2f> displacements;
	vector<bool> vec_validity;
	
	float delta_array_threshold;
	PhaseCorrelation phc;
	vector<Size2i> shapes;
	vector<Point2i> prev_origins, curr_origins;


	// Draw parameters
	int motion_magnification_;
	float gray_img_gain_;
	int gray_img_offset_; 

	Point2f displacement;
	float roll_angle, pitch_angle, yaw_angle;
public:
	
	PYREstimation(){}
	// PYREstimation(float tukey_param = 0.5, float delta_array_threshold= 0.5);
	PYREstimation(const boost::property_tree::ptree config, cv::Mat K);
	~PYREstimation();

	void setFFTReuse(bool reuse_prev_fft) { reuse_prev_fft_ = reuse_prev_fft; }
	void setIntrinsics(Mat intrinsics);
	Point2f getDisplacement() { return displacement; }
	vector<Point2f> getDisplacements() { return displacements; }
	double roll()  { return static_cast<double>(roll_angle); }
	double pitch() { return static_cast<double>(pitch_angle); }
	double yaw()   { return static_cast<double>(yaw_angle); }

	void estimate(Mat image_previous, Mat image_current);
	void estimate(Mat image_previous, Mat image_current, bool reuse_prev_fft,  float thresh_structure = 2, float thresh_distance = 7);

	float estimateRoll(vector<cv::Point2f> displacements, vector<bool> validity, Point2f &displacement, float &distance);
	pair<float, float> estimatePitchYaw(Mat intrinsics, cv::Point2f displacement);
	Mat getRotationMatrix(float roll, float pitch, float yaw);
	void set_windows(cv::Point2i center_windows, int number_windows, int width, int height, int spacing);

	//drawing functions
	Mat drawWindows(Mat img);
	Mat drawCrosshair(Mat img);
	Mat drawDisplacements(Mat img);
	Mat drawMotionVector(Mat img, float magnification = 4);
	Mat draw(Mat img);
	Mat draw(Mat img, float magnification, float graygain, float offset);
	Mat drawGrayImage(Mat img, float gain = 0.5, float offset = 128);
};



/**
Constructor of a PYREstimation object
@param tukey_param float Tukey window ratio
*/
// PYREstimation::PYREstimation(float tukey_param, float delta_array_threshold){	
// 	this->tukey_param = tukey_param;
// 	this->delta_array_threshold = delta_array_threshold;
// 	phc = PhaseCorrelation(this->tukey_param, this->delta_array_threshold );
// }


PYREstimation::PYREstimation(const boost::property_tree::ptree config, cv::Mat K = cv::Mat())
: motion_magnification_(4)
, gray_img_gain_(0.5)
, gray_img_offset_(128) 
, reuse_prev_fft_(false)
, thresh_structure_(2)
, thresh_distance_(7)
, tukey_param(0.5)
, delta_array_threshold(0.5)
{	
	int number_windows_tmp, wind_width_tmp, wind_height_tmp, wind_spacing_tmp;
	cv::Point2i center_windows_tmp = cv::Point2i();

	tukey_param 		  = config.get< float >("phase_correlation.tukey_r");
	delta_array_threshold = config.get< float >("phase_correlation.delta_array_threshold");

	center_windows_tmp.x = config.get< int >("phase_correlation.center_windows_x");
	center_windows_tmp.y = config.get< int >("phase_correlation.center_windows_y");
	number_windows_tmp 	 = config.get< int >("phase_correlation.number_windows");
	wind_width_tmp 		 = config.get< int >("phase_correlation.wind_width");
	wind_height_tmp 	 = config.get< int >("phase_correlation.wind_height");
	wind_spacing_tmp 	 = config.get< int >("phase_correlation.wind_spacing");
	thresh_structure_ 	 = config.get< int >("phase_correlation.thresh_structure");
	thresh_distance_     = config.get< int >("phase_correlation.thresh_distance");

	motion_magnification_ = config.get< int >("drawing_parameters.motion_magnification");
	gray_img_gain_ 		  = config.get< float >("drawing_parameters.gray_img_gain");
	gray_img_offset_ 	  = config.get< int >("drawing_parameters.gray_img_offset");

	Mat intrinsics;
	K.convertTo(intrinsics, CV_32F);

	set_windows(center_windows_tmp, number_windows_tmp, wind_width_tmp, wind_height_tmp, wind_spacing_tmp);
	setIntrinsics(intrinsics);
}

/** Create a set of windows horizontally distributed
@param center_windows Point2i center of the multiple windows
@param number_windows int total number of windows
@param width int width of each window
@param height int height of each window
@param spacing int spacing between windows
 */
void PYREstimation::set_windows(Point2i center_windows, int number_windows, int width, int height, int spacing){
    this->center_windows = center_windows;
    this->number_windows = number_windows;
    vector<vector<int> > windows;
    this->wind_width = width;
    this->wind_height = height;
    
    //phase correlation
    this->shapes.resize(this->number_windows);
    this->prev_origins.resize(this->number_windows);
    this->curr_origins.resize(this->number_windows);
        
    for (int i = 0; i < number_windows; i++){
	//determine the rectangle of the window
	int tl_x = center_windows.x - spacing *(number_windows / 2 - i) - width / 2;
	int tl_y = center_windows.y - height / 2;
	int arr[] = { tl_x, tl_y, width, height };
	vector<int> window(arr, arr + sizeof(arr) / sizeof(arr[0]));
	this->windows.push_back(window);
	// determine the centers of each window
	int center_x = tl_x + width / 2;
	int center_y = tl_y + height / 2;
	this->wind_centers.push_back(Point2i(center_x, center_y));

	//Compensation comp = Compensation(window, this->tukey_param);	  
	//vec_compensation.push_back(comp);
	
	this->shapes[i]= cv::Size2i(width,height);
	this->prev_origins[i] = cv::Point2i(tl_x,tl_y);
	this->curr_origins[i] = cv::Point2i(tl_x,tl_y);
    }  
}

/**
Set the intrinsic parameters for computing the Pitch and Yaw angles
@param intrinsics cv::Mat(3,3,float) intrinsic calibration matrix
**/
void PYREstimation::setIntrinsics(Mat intrinsics){
	this->intrinsics = intrinsics.clone();
}


/** Destructor
 * TODO Release memory
 * 
 */
PYREstimation::~PYREstimation(){

}


/** Estimate the illumination and motion parameters
@param image_previous cv::Mat(M,N,uint8) image for the previous frame
@param image_current cv::Mat(M,N,uint8) image for the current frame
@param reuse_prev_fft bool Flag for reusing the FFT for image_previous from the last call
@param thresh_structure float Minimum pixel variance per pixel to consider that there image has enough structure
@param thresh_distance float Maximum distance value to flag valid motion vector
*/
void PYREstimation::estimate(Mat image_previous, Mat image_current)
{
	estimate(image_previous, image_current, reuse_prev_fft_, thresh_structure_, thresh_distance_);
}


void PYREstimation::estimate(Mat image_previous, Mat image_current, bool reuse_prev_fft, float thresh_structure, float thresh_distance){
    vector<Point2f> displacements;
    vector<bool> vec_validity;

    Mat peaks_image, peaks_mosaic;
    float compactness = 0;
    Rect wind_rect;    

    phc.compute_phase_correlations(image_previous, image_current, this->shapes, this->prev_origins, this->curr_origins, false, reuse_prev_fft);
    vector<PhaseCorrelation::check> checks = phc.get_checks();
    vector<vector<cv::Point2f>> list_displacements = phc.get_displacements();
    for (int w = 0; w < this->number_windows; w++){
	vec_validity.push_back( checks[w].structure_val_0 && checks[w].structure_val_1);
	displacements.push_back(-list_displacements[w][0]);
    }    
   // TODO: reject depending distance	    
    float map_distance;
    Point2f mean_displacement;
    this->roll_angle = this->estimateRoll(displacements, vec_validity, mean_displacement, map_distance);    
    bool valid_distance = map_distance<thresh_distance;
    float min_mapping_dist = 1000000, min_roll;
    // use leave-one-out strategy to find a proper mapping
    if (!valid_distance && !isnan(map_distance)){
	
	Point2f min_displacement;
	int index_min_dist = 0;
	for (int w = 0; w < this->number_windows; w++){
	    vector<bool> vec_validity2 = vec_validity;
	    vec_validity2[w] = false;
	    min_roll = this->estimateRoll(displacements, vec_validity2, mean_displacement, map_distance);
	    // use the computed parameters for the mapping with lowest distance
	    if (map_distance < min_mapping_dist){
		min_mapping_dist = map_distance;
		min_displacement = mean_displacement;
		this->roll_angle = min_roll;
		index_min_dist = w;
	    }
	}
	valid_distance = min_mapping_dist < thresh_distance;
	mean_displacement = min_displacement;
	vec_validity[index_min_dist] = false;
	
    }
    this->pitch_angle = NAN;
    this->yaw_angle = NAN;
    
    // compute pitch and yaw if we know the intrinsics parameters
    if (!this->intrinsics.empty()){
	pair<float, float> pitch_yaw_angle = estimatePitchYaw(intrinsics, mean_displacement);
	this->pitch_angle = pitch_yaw_angle.first;
	this->yaw_angle = pitch_yaw_angle.second;
	Mat rot_matrix = getRotationMatrix(this->roll_angle, this->pitch_angle, this->yaw_angle);
    }
    this->displacements = displacements;
    this->displacement = mean_displacement;
    this->vec_validity = vec_validity;
}



/**
Computes the Roll angle from the windows motion vectors 
@param displacements vector<cv::Point2f> vector with the displacements for each window
@param validity vector<bool> flag indicating for each window its validity
@param mean_displacement Point2f Input/output Global displacement detected in the image
@param distance float Input/output Distance between the aligned pointsets ( displcaments and windows_center)
@return float Roll angle in radians
*/
float PYREstimation::estimateRoll(vector<cv::Point2f> displacements, vector<bool> validity, Point2f &mean_displacement, float &distance){

    int sum_valid = 0;
    for (int i = 0; i < validity.size(); i++){
	sum_valid += (int)validity[i];
    }
    Mat A_mat = Mat(2, sum_valid, CV_32F);
    Mat B_mat = Mat(2, sum_valid, CV_32F);
    Mat A_prime = Mat(2, sum_valid, CV_32F);
    Mat B_prime = Mat(2, sum_valid, CV_32F);
    //instanciate A_mat and B_mat with the windows centers and the motion vectors
    int ind = 0;
    float dx = 0, dy = 0;
    for (int i = 0; i < displacements.size(); i++){
	if (validity[i]){
	    A_mat.at<float>(0, ind) = this->wind_centers[i].x;
	    A_mat.at<float>(1, ind) = this->wind_centers[i].y;
	    B_mat.at<float>(0, ind) = this->wind_centers[i].x + displacements[i].x;
	    B_mat.at<float>(1, ind) = this->wind_centers[i].y + displacements[i].y;
	    dx += displacements[i].x / sum_valid;
	    dy += displacements[i].y / sum_valid;
	    ind = ind + 1;
	}
    }
    float roll;
    if (sum_valid >= 2){
	mean_displacement = Point2f(dx, dy);
    }	else{		
	mean_displacement = Point2f(NAN, NAN);
    }
    if (sum_valid >= 3){	    
	//center the points sets by substracting the mean
	Mat mean_rowA, mean_rowB;
	reduce(A_mat, mean_rowA, 1, CV_REDUCE_AVG);
	reduce(B_mat, mean_rowB, 1, CV_REDUCE_AVG);
	for (int i = 0; i < A_mat.rows; i++){
	    for (int j = 0; j < A_mat.cols; j++){
		A_prime.at<float>(i, j) = A_mat.at<float>(i, j) - mean_rowA.at<float>(i, 0);
		B_prime.at<float>(i, j) = B_mat.at<float>(i, j) - mean_rowB.at<float>(i, 0);
	    }
	}
	//compute roll rotation
	Mat M = A_prime * B_prime.t();
	Mat w, u, vt;
	SVD::compute(M, w, u, vt);
	u.at<float>(1, 1) = -u.at<float>(1, 1);
	Mat roll_rot = u*vt.t();
	roll = -atan2(roll_rot.at<float>(1, 0), roll_rot.at<float>(0, 0));

	//Compute distance between pointsets after applying estimated transformation
	float data[9] = { cos(roll), -sin(roll), dx, sin(roll), cos(roll), dy, 0, 0, 1 };
	Mat T = Mat(3, 3, CV_32F, &data);
	distance = 0;	
	for (int i = 0; i < A_mat.cols; i++){
	    float point[3] = { A_mat.at<float>(0, i), A_mat.at<float>(1, i), 1 };
	    Mat A_p = Mat(3, 1, CV_32F, &point);
	    Mat B_hat = T *A_p;
	    distance += sqrt(pow(B_hat.at<float>(0, 0) - B_mat.at<float>(0, i), 2) + pow(B_hat.at<float>(1, 0) - B_mat.at<float>(1, i), 2));
	}
	distance = distance / sum_valid;
    }else{
	roll = NAN;
	distance = NAN;
    }
    return roll;
}

/**
Computes the pitch and yaw angle given the global displacement and the intrinsic parameters
@param intrinsics cv::Mat(3,3,float) Intrinsic calibration parameters
@param displacement Point2f global displacment
@return pair<float,float> Pitch and Yaw angle in radians
*/
pair<float, float> PYREstimation::estimatePitchYaw(Mat intrinsics, cv::Point2f displacement){
    Point2f p1 = Point2f(this->center_windows.x + displacement.x, this->center_windows.y + displacement.y);
    Point2f p2 = this->center_windows;
	
    float yaw = -atan2(intrinsics.at<float>(0, 2) - p2.x, intrinsics.at<float>(0, 0)) + atan2(intrinsics.at<float>(0, 2) - p1.x, intrinsics.at<float>(0, 0));
    float pitch = atan2(intrinsics.at<float>(1, 2) - p2.y, intrinsics.at<float>(1, 1)) - atan2(intrinsics.at<float>(1, 2) - p1.y, intrinsics.at<float>(1, 1));

    return make_pair(pitch, yaw);
}


/**
Computes the Rotation Matrix given the individual rotationa angles
@param roll float roll rotation angle
@param pitch float pitch rotation angle
@param yaw float yaw rotation angle
@return cv::Mat(3,3,float) returns the rotation matrix
*/
Mat PYREstimation::getRotationMatrix(float roll, float pitch, float yaw){
    //create rotation matrix for each angle
    double data[9] = { cos(-roll), -sin(-roll), 0, sin(-roll), cos(-roll), 0, 0, 0, 1 };
    double data2[9] = { cos(-yaw), 0, sin(-yaw), 0, 1, 0, -sin(-yaw), 0, cos(-yaw) };
    double data3[9] = { 1, 0, 0, 0, cos(-pitch), -sin(-pitch), 0, sin(-pitch), cos(-pitch) };
    Mat mat_roll = Mat(3, 3, CV_64F, &data);
    Mat mat_yaw = Mat(3, 3, CV_64F, &data2);
    Mat mat_pitch = Mat(3, 3, CV_64F, &data3);

    //compute joint rotation matrix
    Mat rot_matrix = mat_roll*(mat_yaw*mat_pitch);
    return rot_matrix;
}

////////////////////DRAWING FUNCTIONS //////////////////////////

Mat PYREstimation::draw(Mat img){
	Mat img_disp = img;
    // img_disp = this->drawGrayImage(img_disp, gray_img_gain_, gray_img_offset_);
	// img_disp = drawWindows(img_disp);
    img_disp = this->drawCrosshair(img_disp);
    img_disp = this->drawMotionVector(img_disp, motion_magnification_);
    return img_disp;
}

Mat PYREstimation::draw(Mat img, float magnification, float gain, float offset){
    Mat img_disp = this->drawGrayImage(img, gain, offset);
    img_disp = this->drawCrosshair(img_disp);
    img_disp = this->drawMotionVector(img_disp, magnification);
    return img_disp;
}


Mat PYREstimation::drawWindows(Mat img){
    for (int i = 0; i < this->number_windows; i++){
	Point2i p1 = Point2i(this->windows[i][0], this->windows[i][1]);
	Point2i p2 = Point2i(this->windows[i][0] + this->wind_width, this->windows[i][1] + this->wind_height);
	rectangle(img, p1, p2, Scalar(0, 255, 0), 2);
    }
    return img;
}

Mat PYREstimation::drawCrosshair(Mat img){
    //draw fixed lines in blue
    line(img, Point2i(img.cols / 2, 0), Point2i(img.cols / 2, img.rows), Scalar(255, 0, 0), 2);
    line(img, Point2i(0, img.rows / 2), Point2i(img.cols, img.rows / 2), Scalar(255, 0, 0), 2);

    //determine edges of moving lines
    float rot_angle;
    if (isnan(this->roll_angle)){
	rot_angle = 0;
    }
    else{
	rot_angle = this->roll_angle;
    }
    float center_vert = img.rows / 2 + this->displacement.y;
    float center_hor = img.cols / 2 + this->displacement.x;
    int left_y = (int)center_hor + center_vert*tan(-rot_angle);
    int right_y = (int)center_hor + center_vert*(-tan(-rot_angle));

    // draw moving lines in red
    img.rows, line(img, Point2i(left_y, 0), Point2i(right_y, img.rows), Scalar(0, 0, 255), 2);
    int top_x = (int)center_vert + center_hor * (-tan(-rot_angle));
    int bottom_y = (int)center_vert + center_hor * tan(-rot_angle);
    line(img, Point2i(0, top_x), Point2i(img.cols, bottom_y), Scalar(0, 0, 255), 2);
    return img;
}

Mat PYREstimation::drawMotionVector(Mat img, float magnification){
    for (int i = 0; i < this->number_windows; i++){
	int ux = int(this->wind_centers[i].x);
	int uy = int(this->wind_centers[i].y);
	int vx = int(ux + displacements[i].x * magnification);
	int vy = int(uy + displacements[i].y * magnification);
	if (this->vec_validity[i]){
	    line(img, Point2i(ux, uy), Point2i(vx, vy), Scalar(255, 0, 0), 2);
	    circle(img, Point2i(ux, uy), 2, Scalar(255, 0, 0), 2);
	}
	else{
	    line(img, Point2i(ux, uy), Point2i(vx, vy), Scalar(0, 0, 255), 2);
	    circle(img, Point2i(ux, uy), 2, Scalar(0, 0, 255), 2);
	}
    }
    return img;
}

Mat PYREstimation::drawGrayImage(Mat img, float gain, float offset){
    gain = 0.5;
    Mat img_disp;
    img.convertTo(img_disp, CV_32FC3);
    img_disp = img_disp * gain + offset;

    //set values lower than 0 to 0
    Mat mask = Mat(img_disp.size(), CV_32FC3);
    threshold(img_disp, mask, 0, 1, CV_THRESH_BINARY);
    img_disp = img_disp.mul(mask);

    //set values larger than 255 to 255
    threshold(img_disp, mask, 255, 1, CV_THRESH_BINARY);
    img_disp = img_disp.mul(Scalar(1, 1, 1) - mask) + mask.mul(255);

    img_disp.convertTo(img, CV_8UC3);
    return img;
}
