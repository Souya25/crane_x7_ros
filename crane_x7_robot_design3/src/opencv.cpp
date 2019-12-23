#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>

using namespace::cv;
std::string msg = "0";
std::string msg_1 = "0"; 
class depth_estimater{
public:
    depth_estimater();
    ~depth_estimater();
    void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
   
 
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_rgb, sub_depth;
    ros::Publisher pub = nh.advertise<std_msgs::String>("bool",100);
};
cv::Mat img_1;
depth_estimater::depth_estimater(){
    //sub_rgb = nh.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 1, &depth_estimater::rgbImageCallback, this);
    sub_rgb = nh.subscribe<sensor_msgs::Image>("/camera/image_raw", 1, &depth_estimater::rgbImageCallback, this);
}
 
depth_estimater::~depth_estimater(){
}
 
void depth_estimater::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg){
 
    cv_bridge::CvImagePtr cv_ptr;
 
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }catch (cv_bridge::Exception& ex){
        ROS_ERROR("error");
        exit(-1);
    }    
    cv::Mat hsv_img;
    cvtColor( cv_ptr->image,hsv_img,CV_BGR2HSV,3);
   
    
    //Scalar lower = cv::Scalar(160,50,50);
    //Scalar upper = cv::Scalar(180,255,255);    
    Scalar lower = cv::Scalar( 0, 200,  200); //フィルタリングする色の範囲
    Scalar upper = cv::Scalar( 50, 255, 255);


	// BGRからHSVへ変換
	Mat mask_image, output_image;
    int px_1,x,y,x_mem,y_mem;
    int flag = 0;
    // inRangeを用いてフィルタリング
	inRange(hsv_img, lower, upper, mask_image);
    int count = 0;
	// マスクを基に入力画像をフィルタリング
	cv_ptr->image.copyTo(output_image, mask_image);
	for( y = 0;y < 480;y++){
        for( x = 0; x < 640; x++){
            px_1 = static_cast<int>(output_image.at<unsigned char>(y, x));  
            if(px_1 > 200 && flag == 0){
               
                x_mem = x;
                y_mem = y;
                flag = 1;
            }
            if(px_1 > 200) count++;
        }
    }
    std::cout << count << std::endl;
    if(flag == 1){
        cv::rectangle(output_image,cv::Point(x_mem-200,y_mem),cv::Point(x_mem-500,y_mem+300),cv::Scalar(0,200,0),3,4);
        //std::cout << x_mem << "," << y_mem << std::endl;
        flag = 0;
    }
    cv::imshow("RGB image", output_image);
    if(count > 3000){
        std::string msg = "1";
        pub.publish(msg);
        msg = "0";
    }

    cv::waitKey(10);
}
int main(int argc, char **argv){
    ros::init(argc, argv, "depth_estimater");
    depth_estimater depth_estimater;
    ros::spin();
    return 0;
}
