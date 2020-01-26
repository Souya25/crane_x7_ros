#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Pose2D.h>
#include <vector>

using namespace::cv;
geometry_msgs::Pose2D ms;
//std::string msg; 
class depth_estimater{
public:          //変数をpublicで宣言
    depth_estimater();
    ~depth_estimater();
    void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
   
 
private:     //hensuu wo private de sengen
    ros::NodeHandle nh;
    ros::Subscriber sub_rgb, sub_depth;
    ros::Publisher pub0 = nh.advertise<geometry_msgs::Pose2D>("bool",1);
    ros::Publisher pub1 = nh.advertise<geometry_msgs::Pose2D>("bool1",1);
    ros::Publisher pub2 = nh.advertise<geometry_msgs::Pose2D>("bool2",1);
    double bunbo = 0;
    double sumx, sumy;
};
cv::Mat img_1;  //int x; mitai na mono //gazou wo kakunou suru hennsuu no sengen 
depth_estimater::depth_estimater(){
    //RealSense
    sub_rgb = nh.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 1, &depth_estimater::rgbImageCallback, this);
	//シミュレータ上のカメラ
    //sub_rgb = nh.subscribe<sensor_msgs::Image>("/camera/image_raw", 1, &depth_estimater::rgbImageCallback, this);
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
   
    
    Scalar lower = cv::Scalar(0,200,200);
    Scalar upper = cv::Scalar(30,255,255);    
    //Scalar lower = cv::Scalar( 0, 120,  120); //フィルタリングする色の範囲
    //Scalar upper = cv::Scalar( 80, 255, 255);


	// BGRからHSVへ変換
	Mat mask_image, output_image;
    int px_1,x,y,x_mem,y_mem;
    int flag = 0;
    // inRangeを用いてフィルタリング
	inRange(hsv_img, lower, upper, mask_image);
    int count = 0;
	
    
    // 輪郭を格納するcontoursにfindContours関数に渡すと輪郭を点の集合として入れてくれる
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(mask_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);    // 輪郭線を格納


    double max_area=0;
    
    int max_area_contour=-1;
    try{
        // 各輪郭をcontourArea関数に渡し、最大面積を持つ輪郭を探す
        for(int j=0; j<contours.size(); j++){
            double area = cv::contourArea(contours.at(j)); //menseki keisan
            if(max_area<area){
                max_area=area;
                max_area_contour=j;
            }
        }
    }catch(CvErrorCallback){
        
    }
    if(max_area_contour != -1){
        int counts = contours.at(max_area_contour).size();
        double gx = 0, gy = 0;    //gx,gyはオブジェクトの重心の座標
    
        for(int k = 0; k < counts; k++){
            gx += contours.at(max_area_contour).at(k).x;
            gy += contours.at(max_area_contour).at(k).y;
        }
         
        gx/=counts;  
        gy/=counts;
        ms.x = -320 + gx;    //ms.xは中心を(0,0)としたときのxの値 (画像の右方向をx軸の正とする)
        ms.y = 240 - gy;    //ms.yは中心を(0,0)としたときのyの値 (画像の上方向をy軸の正とする)
        ms.theta = 1;
        printf("x = %lf, y = %lf theta = %lf\n", ms.x, ms.y, ms.theta);
        pub0.publish(ms);
        pub1.publish(ms);
        pub2.publish(ms);
    }else{
        ms.theta = 0;
        pub0.publish(ms);
        pub1.publish(ms);
        pub2.publish(ms);
        printf("x = %lf, y = %lf theta = %lf\n", ms.x, ms.y, ms.theta);
    } 


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
        }
    }
    if(flag == 1){
        cv::rectangle(output_image,cv::Point(x_mem-200,y_mem),cv::Point(x_mem-500,y_mem+300),cv::Scalar(0,200,0),3,4);
        //std::cout << x_mem << "," << y_mem << std::endl;
        flag = 0;
    }
    
    cv::imshow("RGB image", output_image);
    cv::waitKey(10);
}
int main(int argc, char **argv){
    sleep(2.0);
    ros::init(argc, argv, "depth_estimater");
    depth_estimater depth_estimater;
    ros::spin();
    return 0;
}

