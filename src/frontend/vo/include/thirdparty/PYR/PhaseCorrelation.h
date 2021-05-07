#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>

#include <numeric>

using namespace std;
using namespace cv;


/* This class will run faster if having a Haswell processor and opencv 3.0.0 with AVX2 and SSE intructions set enabled
 */
class PhaseCorrelation{
private: 
    
    float soft_windowing_param;
    int verbose;
    Mat window_function;
    
    int number_of_patches;
    
    
    vector<vector<Point2f>> displacements;
    vector<Mat> covariances;
    vector<Size2i> shapes;
    vector<Point2i> prev_origins,curr_origins;
    bool subpixel_precision;
    vector<float> curr_noise_level;
    
    Mat prev_img,curr_img;
    vector<Mat> curr_patch_fft,curr_patch_mag, curr_patch_phase; 
    vector<bool> curr_structure_check;
    vector<float> curr_structure_val;
    
    Size2i window_size;
    
    float delta_array_threshold;
    
  
public:
    struct check{
      bool delta_array_significance_check;
      bool structure_check;
      float structure_val_0;
      float structure_val_1; 
    };
    vector<check> checks;
  
    PhaseCorrelation(float soft_windowing_param = 0.5, float delta_array_threshold=0.5, int verbose=0);
    vector<vector<Point2f>> get_displacements();
    vector<check>  get_checks();
    vector<Mat> get_covariances();
    
    Mat get_tukey_window(int idx);
    Mat TukeyWindow(int num_elements);
    void compute_phase_correlations(Mat prev_img, Mat curr_img, vector<Size2i> shapes, vector<Point2i> prev_origins, vector<Point2i> curr_origins, bool subpixel_precision=false, bool reuse_prev_fft=false);
    void compute_phase_correlation(int idx, bool reuse_prev_fft=false);
    pair<bool,float> structure_check(Mat &img, float thresh=90);
    Mat fftshift(Mat img);
    vector<Point2f> find_peaks(Mat delta_array);
    float get_noise_level(Mat patch_mag, int n_bins=100, float alpha=0.5);
    pair<Mat,Mat> threshold_spectrum(Mat spectrum, float threshold_val = 250);
  /*  Mat cosine(Mat img);
    Mat sine(Mat img);*/
    
/*    

    Mat get_peak_locations(Mat delta_array, float noise_level, int max_k=5, float alhpa = 0.4, float a=0.6);
    */
    
};

PhaseCorrelation::PhaseCorrelation( float soft_windowing_param, float delta_array_threshold, int verbose){
    this->soft_windowing_param = soft_windowing_param;
    this->delta_array_threshold = delta_array_threshold;
    this->verbose = verbose; 
    this->number_of_patches = 0;
}

/** Returns the checks for each window
 *  @return vector<struct check> Vector with N=num_windows with all the check results
 */ 
vector<PhaseCorrelation::check > PhaseCorrelation::get_checks(){
    return this->checks;
}

/** Returns the covariances for each window
 *  @return vector<Mat> Vector with N=num_windows whoese elements are the covariance matrices
 */ 
vector< Mat > PhaseCorrelation::get_covariances(){
    return this->covariances;
}

/** Returns the displacements for each window
 * @return vector<vector<Point2i>> return the displacements for each window
 */
vector< vector<Point2f> > PhaseCorrelation::get_displacements(){
    return this->displacements;
}

/** Computes a 2D tukey window for the given shape[idx]
 *  @param idx int refers to id of the selected shape
 *  @return  Mat(shape[idx],CV_32) tukey window of size of the shape
 */
Mat PhaseCorrelation::get_tukey_window(int idx){  
    if (this->window_function.rows != this->shapes[idx].height or this->window_function.cols != this->shapes[idx].width ){
	Mat tukey_x = this->TukeyWindow(this->shapes[idx].width);
	Mat tukey_y = this->TukeyWindow(this->shapes[idx].height);
	Mat tx = repeat(tukey_x, this->shapes[idx].height, 1);
	Mat ty = repeat(tukey_y, this->shapes[idx].width, 1);
	ty = ty.t();
	this->window_function = tx.mul(ty);
    }
    return this->window_function;
}


/** Computes a Tukey window of size num_elements
@param num_elements integer size of the Tukey window
@return Mat(1,num_elements,CV_32) tukey window of size num_elements
**/
Mat PhaseCorrelation::TukeyWindow(int num_elements){
    Mat w = Mat(1, num_elements, CV_32FC1);
    const float PI = 3.1415927;
    if (this->soft_windowing_param <= 0){
	w.setTo(1);
    }
    else if (this->soft_windowing_param >= 1){
	//Hanning window		
	createHanningWindow(w, Size(1, num_elements), CV_32F);
    }
    else{
	float spacing = float(1) / (num_elements - 1);
	float per = this->soft_windowing_param / 2;
	float tl = floor(per * (num_elements - 1)) + 1;
	float th = num_elements - tl - 1;
	float val = 0;
	for (int i = 0; i < num_elements; i++){
	    if (i < tl){
		w.at<float>(0, i) = (1 + cos(PI / per*(val - per))) / 2;
	    }
	    else if (i <= th){
		w.at<float>(0, i) = 1;
	    }
	    else{
		w.at<float>(0, i) = (1 + cos(PI / per*(val - 1 + per))) / 2;
	    }
	    val += spacing;
	}
    }
    return w;
}

/**
Checks if the image has enough structure to be used on PC
@param img cv::Mat(M,N,uint8) input image
@param threshold float minimum pixel variance necessary
@return bool true if the image has enough structure
*/
pair< bool, float > PhaseCorrelation::structure_check(Mat &img, float thresh){
      //apply the windowing function
      float mk, pixel_variance;
      
      Mat wind_img = this->window_function.mul(img);
      /**version1- takes 1800 to 300 microsec **/ 
      /*//compute the mean value
      mk = sum(wind_img)[0] / sum(this->window_function)[0];
      //compute the variance
      pow((img - mk), 2.0, img);
      pixel_variance = sum(this->window_function.mul(img))[0] / sum(this->window_function)[0];
      */
      /**version2- takes 800 to 120 microsec **/ 
      Scalar     mean;
      Scalar     stddev;

      cv::meanStdDev ( wind_img, mean, stddev );
      mk = mean.val[0];
      pixel_variance = pow(stddev.val[0],2);
      return std::make_pair( pixel_variance> thresh,pixel_variance);
}

/** Compute for each window the phase correlation
 * @param prev_img
 * @param curr_img
 * @param shapes
 * @param prev_origins
 * @param curr_origins
 * @param subpixel_precision
 * @param reuse_prev_fft
 */
void PhaseCorrelation::compute_phase_correlations(Mat prev_img, Mat curr_img, vector<Size2i> shapes, vector<Point2i> prev_origins, vector<Point2i> curr_origins, bool subpixel_precision, bool reuse_prev_fft){
      


      if (reuse_prev_fft){
	  this->prev_img = this->curr_img.clone();
      }else{
	  prev_img.convertTo(this->prev_img, CV_32F);
      }      
      curr_img.convertTo(this->curr_img, CV_32F);
      this->displacements.clear();
      
      this->shapes = shapes;
      this->prev_origins = prev_origins;
      if (shapes.size() != this->number_of_patches){
	  this->number_of_patches = shapes.size();
	  this->checks.resize(this->number_of_patches);
	  this->curr_noise_level.resize(this->number_of_patches);
	  this->curr_patch_phase.resize(this->number_of_patches);
	  this->curr_patch_mag.resize(this->number_of_patches);
	  this->curr_patch_fft.resize(this->number_of_patches);	  
	  this->curr_structure_check.resize(this->number_of_patches);	  
	  this->curr_structure_val.resize(this->number_of_patches);  
      }
            
      if (prev_origins.size()!=curr_origins.size()){
	  this->curr_origins = prev_origins;
      }else{
	  this->curr_origins = curr_origins;
      }
      for (int i=0;i<shapes.size();i++){
	  this->compute_phase_correlation(i,reuse_prev_fft);	
      }
}

    
/** Compute phase correlation for the selected window
 * @param idx
 * @param reuse_prev_fft
 */
void PhaseCorrelation::compute_phase_correlation(int idx, bool reuse_prev_fft){
    //Select ROI from the patches   
    Mat prev_patch,curr_patch;
    Rect prev_rect = Rect(this->prev_origins[idx].x, this->prev_origins[idx].y, this->shapes[idx].width, this->shapes[idx].height);
    Rect curr_rect = Rect(this->curr_origins[idx].x, this->curr_origins[idx].y, this->shapes[idx].width, this->shapes[idx].height);
    prev_patch = this->prev_img(prev_rect);      
    curr_patch = this->curr_img(curr_rect);
        
     //compute soft windowing matrix
    Mat window_function = this->get_tukey_window(idx);    
    pair<bool,float> curr_structure_results = this->structure_check(curr_patch);    
      
    //subtract mean value of both patches (unbiased patches) and apply the softwind 
    Mat curr_patch_unbiased = curr_patch - cv::mean(curr_patch)[0];       
    curr_patch_unbiased = this->window_function.mul(curr_patch_unbiased);
    
    //compute fft
    Mat curr_patch_fft;
    cv::dft(curr_patch_unbiased, curr_patch_fft, cv::DFT_COMPLEX_OUTPUT);
	  
    //shifting the DC component to be in the middle instead of the corners
    Mat curr_patch_fft_shifted = this->fftshift(curr_patch_fft);  // TODO: remove unnecessary pre/post -shift
    Mat prev_patch_fft,prev_patch_fft_shifted;
    pair<bool,float> structure_prev_results;
    if (reuse_prev_fft){
        prev_patch_fft_shifted = this->curr_patch_fft[idx];
	structure_prev_results.second = this->checks[idx].structure_val_1;
    }else{
      	structure_prev_results = this->structure_check(prev_patch);	
	Mat prev_patch_unbiased = prev_patch - float(cv::mean(prev_patch)[0]);	
	
	prev_patch_unbiased.convertTo(prev_patch_unbiased,CV_32F);
	prev_patch_unbiased = this->window_function.mul(prev_patch_unbiased);
	
	//compute fft
	cv::dft(prev_patch_unbiased, prev_patch_fft, DFT_COMPLEX_OUTPUT);
	
	//shifting that the DC component is the middle instead of the top left corner
	prev_patch_fft_shifted = this->fftshift(prev_patch_fft);	
    }
    // add check results
    this->checks[idx].structure_check = true;
    this->checks[idx].structure_val_0 = structure_prev_results.second;
    this->checks[idx].structure_val_1 = curr_structure_results.second;
    
    //store current results for next iteration
    this->curr_patch_fft[idx] = curr_patch_fft_shifted;
    this->curr_structure_check[idx] = curr_structure_results.first;
    this->curr_structure_val[idx] = curr_structure_results.second;
    
   
    //change to magnitude and phase
    Mat prev_patch_mag, prev_patch_phase;
    vector<Mat> vec_mat; 
     
    if (reuse_prev_fft){
      prev_patch_mag = this->curr_patch_mag[idx].clone();
      prev_patch_phase = this->curr_patch_phase[idx].clone();
    }else{
      split(prev_patch_fft_shifted, vec_mat);    
      cartToPolar(vec_mat[0],vec_mat[1], prev_patch_mag, prev_patch_phase);
    }       
    split(curr_patch_fft_shifted, vec_mat);    
    cartToPolar(vec_mat[0], vec_mat[1], this->curr_patch_mag[idx], this->curr_patch_phase[idx]);

    //get noise level
    float prev_noise_level;
    if (reuse_prev_fft){
	prev_noise_level = this->curr_noise_level[idx];
    }else{
	prev_noise_level = this->get_noise_level(prev_patch_mag);
    }
    this->curr_noise_level[idx] = this->get_noise_level(this->curr_patch_mag[idx]);
    
    //surpress spectral components of negligible magnitude  
    pair<Mat,Mat> prev_thresholded = this->threshold_spectrum(prev_patch_mag, prev_noise_level);
    pair<Mat,Mat> curr_thresholded = this->threshold_spectrum(this->curr_patch_mag[idx], this->curr_noise_level[idx]);
    Mat ones_mat, cross_spectrum,phase_diff = prev_patch_phase - this->curr_patch_phase[idx];
         

    ones_mat = Mat::ones(phase_diff.size(), phase_diff.type());
    Mat sine_mat, cosine_mat;
    polarToCart(ones_mat, phase_diff, cosine_mat,sine_mat);
    vec_mat.clear();
    //vec_mat.push_back( this->cosine(phase_diff));
    //vec_mat.push_back( this->sine(phase_diff));
    vec_mat.push_back(cosine_mat);
    vec_mat.push_back(sine_mat);    
    merge(vec_mat,cross_spectrum);
    Mat all_good_idx,all_bad_idx, delta_array;
    (prev_thresholded.second).convertTo(prev_thresholded.second,CV_8U);
    (curr_thresholded.second).convertTo(curr_thresholded.second,CV_8U);
    bitwise_and(prev_thresholded.second, curr_thresholded.second, all_good_idx);
    
    Mat zero_mat = Mat::zeros(cross_spectrum.size(),cross_spectrum.type());
    cross_spectrum.copyTo(zero_mat, all_good_idx);
    cross_spectrum = zero_mat; 
    
    dft(cross_spectrum, delta_array, DFT_INVERSE |DFT_SCALE, 0);

    //obtain deltaArray
    split(delta_array, vec_mat);
    delta_array = vec_mat[0];
    magnitude(vec_mat[0],vec_mat[1],delta_array);
    delta_array = fftshift(delta_array);
    
    // check delta array information
    Mat da_energy, mask;
    cv::pow(abs(delta_array),2,da_energy);
    int mean_window_size = (delta_array.cols+delta_array.rows)/2;
    float bins1 = this->delta_array_threshold/mean_window_size;
    cv::threshold(da_energy, mask, bins1, 1 , CV_THRESH_BINARY);
    float da_energy_hist0 = sum(mask)[0];
    float da_noise_level = sqrt(bins1);    
    
    //check if there is information in the delta array
    if( da_energy_hist0 < (curr_patch.rows*curr_patch.cols) ){
	this->checks[idx].delta_array_significance_check = true;
    }else{
	this->checks[idx].delta_array_significance_check = false;
    }
        
    this->displacements.push_back(this->find_peaks(delta_array));
    // TODO: Continue with clustering and getting the peaks
    //delta_array.convertTo(delta_array,CV_8U);
        
    //waitKey(0);
}


vector<Point2f> PhaseCorrelation::find_peaks(Mat delta_array){
	vector<Point2f> list_peaks;
    	double max_val, min_val;
	cv::Point min_loc, max_loc;
	minMaxLoc(delta_array, &min_val, &max_val, &min_loc, &max_loc);
	Point2i  p = Point2i(max_loc.x - delta_array.cols/2, max_loc.y - delta_array.rows / 2);
	list_peaks.push_back(p);
	return list_peaks;
}
/*
// TODO: NO longer necessary
Mat PhaseCorrelation::sine(Mat img){
    Mat sin_mat = Mat::zeros(img.size(), img.type());      
    int nElements = img.rows * img.cols; 
    for( int i = 0 ; i < nElements ; ++i ){
        sin_mat.at<float>(i) = sin(img.at<float>(i));
    }
    return sin_mat;
}

// TODO: NO longer necessary
Mat PhaseCorrelation::cosine(Mat img){
    
    
    Mat cos_mat = Mat::zeros(img.size(), img.type());      
    //----------------------------------0.7 - 1ms
    int nElements = img.rows * img.cols; 
    for( int i = 0 ; i < nElements ; ++i ){
        cos_mat.at<float>(i) = cos(img.at<float>(i));
	
    }

    //------------------------------- 2.8 ms
    /*
    int channels = img.channels();

    int nRows = img.rows;
    int nCols = img.cols * channels;

    if (img.isContinuous() and cos_mat.isContinuous())
    {
        nCols *= nRows;
        nRows = 1;
    }

    int i,j;
    uchar* p;
    float * q;
    for( i = 0; i < nRows; ++i)
    {
        p = img.ptr<uchar>(i);
	q = cos_mat.ptr<float>(i);
        for ( j = 0; j < nCols; ++j)
        {
            q[j] = cos(p[j]);
        }
    }
  
    return cos_mat;
}
*/
/** Threhsold the spectrum given a threhs val
 * @param specturm Mat<CV_32F> spectrum matrix
 * @param threshold_val float threshold value 
 * @return pair<Mat,Mat> return the thresholded spectrum and a mask
 */
pair<Mat,Mat> PhaseCorrelation::threshold_spectrum(Mat spectrum, float threshold_val){
	Mat mask, spectrum2 = spectrum.clone();
	threshold(spectrum, mask, threshold_val, 1, CV_THRESH_BINARY);	
	mask.convertTo(mask,CV_8U);
        spectrum2.setTo(0,mask);
	return make_pair(spectrum2,mask);
}

/** Measure the noise level of the magnitude of the spectrum
 * @param patch_mag Mat image with the magnitude of the spectrum
 * @param n_bins int number of bins of the histogram
 * @param alpha float multiplicative factor for the threhsold
 * @return float witht the noise level
 */
float PhaseCorrelation::get_noise_level(Mat patch_mag, int n_bins, float alpha){  
     Mat hist,mask;
      int histSize[] = {n_bins};
      float range[] = { 0, 5000 } ;
      const float* ranges = { range };
      int index = n_bins;
      calcHist( &patch_mag, 1, 0, Mat(), hist, 1, histSize, &ranges);
      float mag_hist_sum = sum(hist)[0];
    
      float cumsum=0;
      for (int i=0;i<n_bins;i++){
	  cumsum+= hist.at<float>(i);
	  if(cumsum>mag_hist_sum*alpha){
	      index = i+1;
	      break;
	  }
      }      
      
      float current_bin = range[0] + (index+1) * range[1]/(n_bins);
      threshold(patch_mag, mask, current_bin, 1, CV_THRESH_BINARY_INV);
      float mean_val = 2 * sum(patch_mag.mul(mask))[0]/sum(mask)[0];
      return mean_val;
}

/** Performs the inverse and forward fftshift, only for size multiple of 2
 *  @param img Mat input image of anytype
 *  @return Mat with the shifted content swaping the diagonal quadrants
 */
Mat PhaseCorrelation::fftshift(Mat img){
	assert((img.cols%2)==0 and (img.rows%2)==0 && "The patch size must be divisible by 2 in any dimensions");
	Mat quadrant;
	Mat dst = Mat::zeros(img.size(), img.type());
	Mat patch = Mat::zeros(img.rows / 2, img.cols / 2, img.type());
	//swap top left by bottom right
	img(Rect(0, 0, img.cols / 2, img.rows / 2)).copyTo(patch);
	patch.copyTo(dst(Rect(img.cols / 2, img.rows / 2, patch.cols, patch.rows)));
	//swap top right by bottom left
	img(Rect(img.cols / 2, 0, img.cols / 2, img.rows / 2)).copyTo(patch);
	patch.copyTo(dst(Rect(0, img.rows / 2, patch.cols, patch.rows)));
	//swap bottom right by top left
	img(Rect(0, img.rows / 2, img.cols / 2, img.rows / 2)).copyTo(patch);
	patch.copyTo(dst(Rect(img.cols / 2, 0, patch.cols, patch.rows)));
	//swap bottom left by top right
	img(Rect(img.cols / 2, img.rows / 2, img.cols / 2, img.rows / 2)).copyTo(patch);
	patch.copyTo(dst(Rect(0, 0, patch.cols, patch.rows)));
	return dst;
}
