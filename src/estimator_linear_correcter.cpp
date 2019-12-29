#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <turtlesim/Pose.h>
#include <string>


#define IMAGE_HEIGHT 440
#define IMAGE_WIDTH 640

//畝の領域推定のときの閾値計算の割合
#define THRESHOLD_RATIO 0.9

//#define INPUT_TOPIC "/camera/depth/image_rect_raw"
#define INPUT_TOPIC "/camera/aligned_depth_to_color/image_raw"

cv_bridge::CvImage dimg;

//cameraに映るdepthの最大と最小を求めて、畝かどうかの判別の閾値を作る。
int threshold = 400;

int pic_num = 0;

class depth_estimater{
public:
    depth_estimater();
    ~depth_estimater();
    double center[IMAGE_HEIGHT];//畝の中央の画素の位置を格納する配列。縦は480画素なので
    bool ridge_region[IMAGE_HEIGHT][IMAGE_WIDTH];//畝の領域を示す配列。畝だと認識されたら1、畝じゃなかったら0が入る。
    float center_center;//畝の中央の画素の位置の全部の行の平均。畝の中心線の重心。
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_rgb, sub_depth;
    ros::Publisher center_pub, d_pub;
//    ros::Publisher d_rgb_pub;
    ros::Publisher center_distance_pub, ridge_angle_pub;
    image_transport::Publisher rgb_pub;

    //直線近似の直線引くよう
    double x_0 = 0.;
    double x_max = 0;

};

//コンストラクタ 
depth_estimater::depth_estimater(){

    //ここに入力するトピックを書く
/*
    sub_depth = nh.subscribe<sensor_msgs::Image>("/camera/depth/image_rect_raw", 1, &depth_estimater::depthImageCallback, this);//for gazebo-> /pico_flexx_link/depth_registered/image_raw
    sub_rgb = nh.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 1, &depth_estimater::rgbImageCallback, this);

    sub_depth = nh.subscribe<sensor_msgs::Image>("/pico_flexx_link/depth_registered/image_raw", 1, &depth_estimater::depthImageCallback, this);//for gazebo-> /pico_flexx_link/depth_registered/image_raw
    sub_rgb = nh.subscribe<sensor_msgs::Image>("/pico_flexx_link_ir/image_raw", 1, &depth_estimater::rgbImageCallback, this);

            <param name="input_depth_image_topic" value="/pico_flexx_link/depth_registered/image_raw"/>
            <param name="input_color_image_topic" value="/pico_flexx_link_ir/image_raw"/>

*/
    sub_depth = nh.subscribe<sensor_msgs::Image>(INPUT_TOPIC, 10, &depth_estimater::depthImageCallback, this);//for gazebo-> /pico_flexx_link/depth_registered/image_raw
    sub_rgb = nh.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 10, &depth_estimater::rgbImageCallback, this);


     //open cvの画像をrosトピックとしてパブリッシュするため。
    image_transport::ImageTransport it(nh);
    rgb_pub = it.advertise("/estimator_linear/correction/rgb",10);
    d_pub = nh.advertise<sensor_msgs::Image>("/estimator_linear/correction/d", 10);
//    d_rgb_pub = nh.advertise<sensor_msgs::Image>("/debug/estimator_linear/d_rgb", 10);

}

//デストラクタ
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

    cv::Mat &mat = cv_ptr->image;

    //直線近似の直線
    cv::line(cv_ptr->image, cv::Point(320, 0), cv::Point(320, 480), cv::Scalar(255,255,255), 1, 1);


    cv::imshow("RGB image", cv_ptr->image);

    sensor_msgs::ImagePtr image_rbg_processed = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
    rgb_pub.publish(image_rbg_processed);


    cv::waitKey(10);

}




void depth_estimater::depthImageCallback(const sensor_msgs::ImageConstPtr& msg){
}





int main(int argc, char **argv){

    ros::init(argc, argv, "estimater_correcter");

    depth_estimater depth_estimater;

    ros::spin();
    return 0;
}
