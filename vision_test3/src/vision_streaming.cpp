#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>

using namespace cv;

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";
static const char CANNY_WINDOW[] = "Canny Window";
static const char THRESHOLD_WINDOW[] = "Threshold Window";

void thresh_callback(int, void* );

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
  Mat outputMat;
  int threshCanny;
  int threshVal;
  int max_thresh;
  RNG rng;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    //image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("/wide_stereo/left/image_rect_color", 1, &ImageConverter::imageCb, this);

    threshCanny = 255;
    threshVal = 125;
    max_thresh = 255;
    rng = RNG(12345);

    namedWindow(WINDOW);
    namedWindow(CANNY_WINDOW);
    namedWindow(THRESHOLD_WINDOW);
    
    createTrackbar( " Canny thresh:", CANNY_WINDOW, &threshCanny, max_thresh, thresh_callback );
    createTrackbar( " Thresholding:", THRESHOLD_WINDOW, &threshVal, max_thresh, thresh_callback );
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    //covert the rosimage into an openCV Mat image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    //store the new Mat image inside of a Mat object for easy access
    Mat src = cv_ptr->image;
    
    //break into color spaces for comparison
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
    //cvtColor( src, src_gray, CV_BGR2GRAY );
    //blur( src_gray, src_gray, Size(3,3) );
    
    //set image to be processed
    outputMat = luv[0];
    
    //begin processing the image
    Mat canny_output;
    Mat threshedMat;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    //threshedMat = outputMat;
    threshold(outputMat, threshedMat, threshVal, 255, CV_THRESH_BINARY);

    /// Detect edges using canny
    //canny_output = threshedMat;
    Canny( threshedMat, canny_output, threshCanny, threshCanny*2, 3 );

    /// Find contours
    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );

    for( int i = 0; i < contours.size(); i++ ) { 
        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
    }

    /// Draw contours
    //Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    for( size_t i = 0; i< contours.size(); i++ ) {
        Scalar color = Scalar(0, 0, 255);//( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        //drawContours( drawing, contours, (int)i, color, 2, 8, hierarchy, 0, Point() );
        drawContours( src, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        rectangle( src, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
        circle( src, center[i], 1, color, 2, 8, 0 );
        //circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
    }
    
    //draws a small circle on the image to signify it has been converted to openCV Mat format
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    imshow(WINDOW, src); //cv_ptr->image);
    imshow(CANNY_WINDOW, canny_output);
    imshow(THRESHOLD_WINDOW, threshedMat);
    waitKey(3);
    
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
};

void thresh_callback(int, void* )
{
}

int main(int argc, char** argv)
{
  ROS_INFO("===========Entered main===========\n");
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
