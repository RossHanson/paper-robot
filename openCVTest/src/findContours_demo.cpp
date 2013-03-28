/**
 * @function findContours_Demo.cpp
 * @brief Demo code to find contours in an image
 * @author OpenCV team
 */

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include <cxcore.h>
//#include <ml.h>
#include <cv.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

Mat src;
Mat src_gray;
//IplImage* outputIpl;
//IplImage* threshedOutput;
//IplImage* originalIpl;
Mat outputMat;
int threshCanny = 255;
int threshVal = 125;
int red = 100;
int green = 100;
int blue = 100;
int max_thresh = 255;
int max_color = 255;
RNG rng(12345);
const char* outputWindow;

/// Function header
void thresh_callback(int, void* );

/**
 * @function main
 */
int main( int, char** argv )
{
  // Load source image and convert it to gray
  src = imread( argv[1], 1 );  
  
  //create 3 seperate color spaces (9 color channels)
  //rgb
  vector<Mat> rgb;
  split(src, rgb);  
  
  //luv
  Mat luvMat;
  vector<Mat> luv;
  cvtColor( src, luvMat, CV_BGR2Luv );
  split(luvMat, luv);
  
  //hsv
  Mat hsvMat;
  vector<Mat> hsv;
  cvtColor( src, hsvMat, CV_BGR2HSV );
  split(hsvMat, hsv);
  
  //blur the channels
  blur( rgb[0], rgb[0], Size(3,3) );
  blur( rgb[1], rgb[1], Size(3,3) );
  blur( rgb[2], rgb[2], Size(3,3) );
  
  blur( luv[0], luv[0], Size(3,3) );
  blur( luv[1], luv[1], Size(3,3) );
  blur( luv[2], luv[2], Size(3,3) );
  
  blur( hsv[0], hsv[0], Size(3,3) );
  blur( hsv[1], hsv[1], Size(3,3) );
  blur( hsv[2], hsv[2], Size(3,3) );
  
  //output
  /*namedWindow( "Red", CV_WINDOW_AUTOSIZE );
  imshow( "Red", rgb[2] );
  namedWindow( "Blue", CV_WINDOW_AUTOSIZE );
  imshow( "Blue", rgb[1] );
  namedWindow( "Green", CV_WINDOW_AUTOSIZE );
  imshow( "Green", rgb[0] );*/
  
  /*namedWindow( "L", CV_WINDOW_AUTOSIZE );
  imshow( "L", luv[2] );
  namedWindow( "U", CV_WINDOW_AUTOSIZE );
  imshow( "U", luv[1] );*/
  namedWindow( "lV", CV_WINDOW_AUTOSIZE );
  imshow( "lV", luv[0] );
  
  /*namedWindow( "H", CV_WINDOW_AUTOSIZE );
  imshow( "H", hsv[2] );
  namedWindow( "S", CV_WINDOW_AUTOSIZE );
  imshow( "S", hsv[1] );
  namedWindow( "hV", CV_WINDOW_AUTOSIZE );
  imshow( "hV", hsv[0] );*/
  
  /// Convert image to gray and blur it
  cvtColor( src, src_gray, CV_BGR2GRAY );
  blur( src_gray, src_gray, Size(3,3) );

  //set image to be processed
  outputMat = luv[0];
  
  /// Create Window
  const char* source_window = "Source";
  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  imshow( source_window, src );
  

  //create trackbars
  createTrackbar( " Canny thresh:", "Source", &threshCanny, max_thresh, thresh_callback );
  //createTrackbar( " Thresholding:", "Source", &threshVal, max_thresh, thresh_callback );
  thresh_callback( 0, 0 );

  waitKey(0);
  
  return(0);
}

/**
 * @function thresh_callback
 */
void thresh_callback(int, void* )
{
  Mat canny_output;
  Mat threshedMat;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  
  //threshedMat = outputMat;
  threshedMat = outputMat;
  //threshold(outputMat, threshedMat, threshVal, 255, CV_THRESH_BINARY);
  
  /// Detect edges using canny
  //canny_output = threshedMat;
  Canny( threshedMat, canny_output, threshCanny, threshCanny*2, 3 );
  /// Find contours
  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Draw contours
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  for( size_t i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, (int)i, color, 2, 8, hierarchy, 0, Point() );
     }

  /// Show in a window
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Canny", canny_output);
  //imshow( "Threshed", threshedMat);
  imshow( "Contours", drawing );
}
