/**
 * @function findContours_Demo.cpp
 * @brief Demo code to find contours in an image
 * @author OpenCV team
 */

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

Mat src; 
Mat src_gray;
int thresh = 100;
int red = 100;
int green = 100;
int blue = 100;
int max_thresh = 255;
int max_color = 255;
RNG rng(12345);

/// Function header
void thresh_callback(int, void* );

/**
 * @function main
 */
int main( int, char** argv )
{
  /// Load source image and convert it to gray
  src = imread( argv[1], 1 );

  //create filtered windows
  vector<Mat> rgb;
  split(src, rgb);
  blur( rgb[0], rgb[0], Size(3,3) );
  blur( rgb[1], rgb[1], Size(3,3) );
  blur( rgb[2], rgb[2], Size(3,3) );
  
  namedWindow( "Red", CV_WINDOW_AUTOSIZE );
  imshow( "Red", rgb[2] );
  namedWindow( "Blue", CV_WINDOW_AUTOSIZE );
  imshow( "Blue", rgb[1] );
  namedWindow( "Green", CV_WINDOW_AUTOSIZE );
  imshow( "Green", rgb[0] );
  

  /// Convert image to gray and blur it
  cvtColor( src, src_gray, CV_BGR2GRAY );
  blur( src_gray, src_gray, Size(3,3) );

  //set image to be processed
  output = rgb[1];
  
  /// Create Window
  const char* source_window = "Source";
  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  imshow( source_window, src );
  

  //create trackbars
  createTrackbar( " Canny thresh:", "Source", &thresh, max_thresh, thresh_callback );
  //createTrackbar( " Red:", "Source", &red, max_color, thresh_callback );
  //createTrackbar( " Green:", "Source", &green, max_color, thresh_callback );
  //createTrackbar( " Blue:", "Source", &blue, max_color, thresh_callback );
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
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using canny
  Canny( src_gray, canny_output, thresh, thresh*2, 3 );
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
  imshow( "Contours", drawing );
}
