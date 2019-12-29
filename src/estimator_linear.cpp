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

#define INPUT_TOPIC "/camera/depth/image_rect_raw"
//#define INPUT_TOPIC "/camera/aligned_depth_to_color/image_raw"

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

    //for gazebo-> /pico_flexx_link_ir/image_raw
    center_pub = nh.advertise<std_msgs::Float32>("/cabbage/center", 10);
    center_distance_pub = nh.advertise<std_msgs::Float32>("/estimator_linear/center_distance", 10);
    ridge_angle_pub = nh.advertise<std_msgs::Float32>("/estimator_linear/ridge_angle", 10);


     //open cvの画像をrosトピックとしてパブリッシュするため。
    image_transport::ImageTransport it(nh);
    rgb_pub = it.advertise("/estimator_linear/rgb",10);
    d_pub = nh.advertise<sensor_msgs::Image>("/estimator_linear/d", 10);
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

///*
    //畝の領域を色付け
    for(int i = 0; i < IMAGE_HEIGHT; i++){
        for(int j = 0 ; j < cv_ptr->image.cols; j++){
            if(ridge_region[i][j]==1){
                cv::line(cv_ptr->image, cv::Point(j, i), cv::Point(j, i), cv::Scalar(0,0,100), 2, 4);
            }
        }
    }
//*/

    //iは行数(縦)。ここでカラー画像の方に畝の中心線を赤く表示している。
    for(int i = 0; i < IMAGE_HEIGHT; i++){
        cv::line(cv_ptr->image, cv::Point(center[i], i), cv::Point(center[i], i), cv::Scalar(0,200,0), 3, 4);  
    }

    //直線近似の直線
    cv::line(cv_ptr->image, cv::Point(x_0, 0), cv::Point(x_max, 480), cv::Scalar(255,0,0), 2, 4);


    cv::imshow("RGB image", cv_ptr->image);

//    cv::waitKey(10); // とりあえず、ここが時間かかるとおかしくなるっぽい

    //publish image_rgb
    sensor_msgs::ImagePtr image_rbg_processed = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
    rgb_pub.publish(image_rbg_processed);


    cv::waitKey(10);

/*
    cv::rectangle(cv_ptr->image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255), 3, 4);
    cv::imshow("RGB image", cv_ptr->image);
    cv::waitKey(10);
*/
}




void depth_estimater::depthImageCallback(const sensor_msgs::ImageConstPtr& msg){


    //水平方向の1行の畝の領域のことをridge_lineと呼ぶことにする。
    //これは、ridge_lineのかず。たまに畝領域が0の行が出るので、その分を除いてカウントするため。
    int num_of_ridge_line = 0;

    float sum_all_i = 0.0;
    double ave;
    cv_bridge::CvImagePtr cv_ptr;//ptrメソッド
    cv_bridge::CvImagePtr cv_ptr2;//ptrメソッド

    //畝の高さとか低さとか
    int min_height = 99999;
    int max_height = 0;

    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);//16UC1にするとコアダンプする。
        cv_ptr2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
    }catch (cv_bridge::Exception& ex){
        ROS_ERROR("error");
        exit(-1);
    }

    //CV_32FC1型の要素を持つ配列depthとCV_8UC1型の要素を持つ配列imgを用意。画素数はcv_ptrと一緒。
    cv::Mat depth(IMAGE_HEIGHT, cv_ptr->image.cols, CV_32FC1);
    cv::Mat img(IMAGE_HEIGHT, cv_ptr->image.cols, CV_8UC1);
    cv::Mat img3(IMAGE_HEIGHT, cv_ptr->image.cols, CV_16UC1);


    /*
    cv::imwrite("my_debug/output.jpg", depth);
    pic_num++;
    */

    //iは行数(縦)
    for(int i = 0; i < IMAGE_HEIGHT;i++){
        float* Dimage = cv_ptr->image.ptr<float>(i);//画像のi行目の先頭画素のポインタを取得
        float* Iimage = depth.ptr<float>(i);//用意した配列のi行目の先頭画素のポインタを取得
        char* Ivimage = img.ptr<char>(i);
        short* Simage = img3.ptr<short>(i);

        float sum_of_one_ridge_line_depth = 0.0;//水平方向の畝領域のdpethの合計。行が変わるたびにリセット。
        int num_of_pixel_in_ridge_line = 0; //水平方向の畝領域のピクセル数。

        //全領域走査
        //jは列(左右方向)
        for(int j = 0 ; j < cv_ptr->image.cols; j++){
            //cameraに映るdepthの最大と最小を求める
            if(Dimage[j]<min_height && Dimage[j]>10){
                min_height = Dimage[j];
            }else if(Dimage[j]>max_height){
                max_height = Dimage[j];
            }

            if(Dimage[j] > 0.0){
                //畝かどうかの判断
                if(Dimage[j] < threshold){//ある距離より近くまで、畝が近づいた時 807良さげ
                    ridge_region[i][j] = 1;//畝だったらtrue
                    sum_of_one_ridge_line_depth += j;
                    num_of_pixel_in_ridge_line++;
                }else{
                    ridge_region[i][j] = 0;//畝じゃなかったらfalse
                }
            }else{
            }
        }

        //img3に代入し直し。本来は16UC1なのにcv_bridge::toCvCopyでうまくいかないので。
        for(int j = 0 ; j < (cv_ptr->image.cols); j++){//j = 0 ; j < cv_ptr->image.cols; j++
            Simage[j] = (short)Dimage[j];
        }

        center[i] = sum_of_one_ridge_line_depth / num_of_pixel_in_ridge_line;//畝の真ん中の画素の位置を出す。
        if(center[i] > 0){
            num_of_ridge_line++;
        }
        sum_all_i = sum_all_i + sum_of_one_ridge_line_depth / num_of_pixel_in_ridge_line;//全部の行の真ん中の画素を足しあわせる。
    }

    threshold = int((max_height + min_height)/2* THRESHOLD_RATIO);//閾値をとりあえず最大と最小の真ん中にする
//    ROS_INFO( "max:%d  min:%d", max_height, min_height);
//    ROS_INFO( "threshold:%d", threshold);


    /***最小二乗法***/
    double a = 0.0;
    double b = 0.0;
//    double n = IMAGE_HEIGHT;//ここエラーの原因。depth拾えてない文も計算してしまってる。
//    double n = num_of_ridge_line; //ここエラーの原因。20191126。nは固定ではない。
//    double n = 0.0; // これもダメっぽい
    double n = 0.0;
//    ROS_INFO("num_of_ridge_line : %8.3lf", num_of_ridge_line);
    double sigma_x, sigma_y, sigma_xy, sigma_xx;
    sigma_x = 0.0;
    sigma_y = 0.0;
    sigma_xy = 0.0;
    sigma_xx = 0.0;

//    ROS_INFO("IMAGE_HEIGHT : %d", IMAGE_HEIGHT);
    for(int i=0; i<IMAGE_HEIGHT; i++){
        if(center[i] > 0.1){//center[i]がnanじゃないときのみ追加
            sigma_xy += (double)i * center[i];
            sigma_x += (double)i;
            sigma_y += center[i];
            sigma_xx += (double)i * (double)i;
            n = n + 1.;
//            ROS_INFO("!center[i] : %8.3lf", center[i]); // ここは問題なさそう
        }
    }
///*

//    cv::waitKey(10);


    ROS_INFO("sigma_x : %8.3lf", sigma_x);
    ROS_INFO("sigma_y : %8.3lf", sigma_y);
    ROS_INFO("sigma_xy : %8.3lf", sigma_xy);
    ROS_INFO("sigma_xx : %8.3lf", sigma_xx);

    ROS_INFO("a_bunshi : %8.3lf", (n*sigma_xy - sigma_x*sigma_y));
    ROS_INFO("b_bunshi : %8.3lf", (sigma_xx*sigma_y - sigma_xy*sigma_x));
    ROS_INFO("bunbo : %8.3lf", (n*sigma_xx - sigma_x*sigma_x));
    ROS_INFO("n*sigma_xx : %8.3lf", n*sigma_xx);
    ROS_INFO("sigma_x*sigma_x : %8.3lf", sigma_x*sigma_x);
    ROS_INFO("n : %8.3lf", n);
//*/
    a = (n*sigma_xy - sigma_x*sigma_y)/(n*sigma_xx - sigma_x*sigma_x);
    b = (sigma_xx*sigma_y - sigma_xy*sigma_x)/(n*sigma_xx - sigma_x*sigma_x);
    ROS_INFO("a : %8.3lf", a);
    ROS_INFO("b : %8.3lf", b);


    x_0 = int(b);
    x_max = int(a*480 + b);
//    ROS_INFO("a: %8.3lf  b: %8.3lf", a, b);
//    ROS_INFO("x_0 : %8.3lf  x_max : %8.3lf", x_0, x_max);

    std_msgs::Float32 center_distance_float32; // TODO　単位どうしよ？
    std_msgs::Float32 ridge_angle_float32; //rad

    ridge_angle_float32.data = (float)atan2(a, 1.);
    double line_center = a * (IMAGE_HEIGHT/2.) + b;// ここはラジアン。atan2の返り値は[-pi, pi]が範囲
    center_distance_float32.data = float( line_center - (cv_ptr->image.cols/2.) ); // この段階だとまだピクセル単位
    
    center_distance_pub.publish(center_distance_float32);
    ridge_angle_pub.publish(ridge_angle_float32);

    /******/


/*
    center_center = sum_all_i/IMAGE_HEIGHT;//畝の中心線の重心を出す。

    std_msgs::Float32 center_float32;
    center_float32.data = center_center;
    center_pub.publish(center_float32);


//    ROS_INFO("center_center : %f [個目]", center_center);
//    ROS_INFO("%f", center_center);

    dimg.encoding = sensor_msgs::image_encodings::MONO16;//----------CHANGED
    dimg.image = img3;//----------CHANGED

    d_pub.publish(dimg.toImageMsg());//----------CHANGED

//    cv::imshow("DEPTH image cv_ptr->image", cv_ptr->image);

    cv::waitKey(10);
*/
}





int main(int argc, char **argv){

    ros::init(argc, argv, "ridge_center_estimater");

    depth_estimater depth_estimater;

    ros::spin();
    return 0;
}
