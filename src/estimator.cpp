#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <turtlesim/Pose.h>

//使ってない
#define WIDTH   50
#define HEIGHT  25

#define image_height 480
#define image_width 640

cv_bridge::CvImage dimg;

class depth_estimater{
public:
    depth_estimater();
    ~depth_estimater();
    int center[image_height];//畝の中央の画素の位置を格納する配列。縦は480画素なので
    bool ridge_region[image_height][image_width];//畝の領域を示す配列。畝だと認識されたら1、畝じゃなかったら0が入る。
    float center_center;//畝の中央の画素の位置の全部の行の平均。畝の中心線の重心。
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_rgb, sub_depth;
    ros::Publisher center_pub, d_pub;//----------CHANGED
    ros::Publisher d_rgb_pub;//----------CHANGED
    image_transport::Publisher rgb_pub;

};

//コンストラクタ 
depth_estimater::depth_estimater(){
    //ここに入力するトピックを書く
    sub_depth = nh.subscribe<sensor_msgs::Image>("/camera/depth/image_rect_raw", 1, &depth_estimater::depthImageCallback, this);//for gazebo-> /pico_flexx_link/depth_registered/image_raw
    sub_rgb = nh.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 1, &depth_estimater::rgbImageCallback, this);
    //for gazebo-> /pico_flexx_link_ir/image_raw
    center_pub = nh.advertise<std_msgs::Float32>("/cabbage/center", 1, 100);

     //open cvの画像をrosトピックとしてパブリッシュするため。
    image_transport::ImageTransport it(nh);
    rgb_pub = it.advertise("/cabbage/rgb",1, 100);
    d_pub = nh.advertise<sensor_msgs::Image>("/cabbage/d", 10);//----------CHANGED
    d_rgb_pub = nh.advertise<sensor_msgs::Image>("/cabbage/d_rgb", 10);//----------CHANGED//こいつはうまくいってない

}

//デストラクタ
depth_estimater::~depth_estimater(){
}


void depth_estimater::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg){

    int i, j;
    int x1, x2, y1, y2;
    int width = WIDTH;
    int height = HEIGHT;
    cv_bridge::CvImagePtr cv_ptr;

    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }catch (cv_bridge::Exception& ex){
        ROS_ERROR("error");
        exit(-1);
    }

    cv::Mat &mat = cv_ptr->image;

/*  //赤い四角形書くとこ
    x1 = int(mat.cols / 2) - width;
    x2 = int(mat.cols / 2) + width;
    y1 = int(mat.rows / 2) - height;
    y2 = int(mat.rows / 2) + height;

    for(i = y1; i < y2; i++){
        for(j = x1; j < x2; j++){
            // 0 : blue, 1 : green, 2 : red.
            mat.data[i * mat.step + j * mat.elemSize() + 0] = 0;
            mat.data[i * mat.step + j * mat.elemSize() + 1] = 0;
            //mat.data[i * mat.step + j * mat.elemSize() + 2] = 0;
        }
    }
*/

    //畝の領域を色付け
    for(i = 0; i < image_height; i++){
        for(j = 0 ; j < cv_ptr->image.cols; j++){
            if(ridge_region[i][j]==1){
                cv::line(cv_ptr->image, cv::Point(j, i), cv::Point(j, i), cv::Scalar(0,0,100), 2, 4);
            }
        }
    }

    //iは行数(縦)。ここでカラー画像の方に畝の中心線を赤く表示している。
    for(i = 0; i < image_height; i++){
        cv::line(cv_ptr->image, cv::Point(center[i], i), cv::Point(center[i], i), cv::Scalar(0,200,0), 3, 4);  
        }
    cv::imshow("RGB image", cv_ptr->image);
    cv::waitKey(10);

    //重心を表示。これに従ってロボットのタイヤの左右の出力差を出す。mass_center未実装
//  cv::line(cv_ptr->image, cv::Point(mass_center, 0), cv::Point(mass_center, 0), cv::Scalar(255,0,0), 2, 4);

    //publish image_rgb
    sensor_msgs::ImagePtr image_rbg_processed = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
    rgb_pub.publish(image_rbg_processed);

/*
    cv::rectangle(cv_ptr->image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255), 3, 4);
    cv::imshow("RGB image", cv_ptr->image);
    cv::waitKey(10);
*/
}

void depth_estimater::depthImageCallback(const sensor_msgs::ImageConstPtr& msg){

    int x1, x2, y1, y2;
    int i, j, k;
    int width = WIDTH;
    int height = HEIGHT;
    float sum_i = 0.0;
    float sum_all_i = 0.0;
    int count = 0;
    double ave;
    cv_bridge::CvImagePtr cv_ptr;//ptrメソッド
    cv_bridge::CvImagePtr cv_ptr2;//ptrメソッド

    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);//16UC1にするとコアダンプする。
        cv_ptr2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);//----------CHANGED
    }catch (cv_bridge::Exception& ex){
        ROS_ERROR("error");
        exit(-1);
    }

    //CV_32FC1型の要素を持つ配列depthとCV_8UC1型の要素を持つ配列imgを用意。画素数はcv_ptrと一緒。
    cv::Mat depth(cv_ptr->image.rows, cv_ptr->image.cols, CV_32FC1);
//    cv::Mat img(cv_ptr->image.rows, cv_ptr->image.cols, CV_16UC1);
    cv::Mat img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);

    cv::Mat img3(cv_ptr->image.rows, cv_ptr->image.cols, CV_16UC1);

//    cv::Mat d_rgb(cv_ptr->image.rows, cv_ptr->image.cols, CV_32FC3);

 
/* 
    x1 = int(depth.cols / 2) - width;
    x2 = int(depth.cols / 2) + width;
    y1 = int(depth.rows / 2) - height;
    y2 = int(depth.rows / 2) + height;
*/

//    int center[cv_ptr->image.rows];//畝の中央の画素の位置を格納する配列。

    //iは行数(縦)
    for(i = 0; i < cv_ptr->image.rows;i++){
        float* Dimage = cv_ptr->image.ptr<float>(i);//画像のi行目の先頭画素のポインタを取得
        float* Iimage = depth.ptr<float>(i);//用意した配列のi行目の先頭画素のポインタを取得
        char* Ivimage = img.ptr<char>(i);
        short* Simage = img3.ptr<short>(i);

        sum_i = 0.0;//sumをリセット。
        count = 0;

        //全領域走査
        for(j = 0 ; j < cv_ptr->image.cols; j++){
            if(Dimage[j] > 0.0){
                //畝かどうかの判断
                if(Dimage[j] < 450){//ある距離より近くまで、畝が近づいた時 807良さげ
                    ridge_region[i][j] = 1;//畝だったらtrue
                }else{
                    ridge_region[i][j] = 0;//畝じゃなかったらfalse
                }
            }else{
            }
        }

        //jで横幅調整して走査
        for(j = 0 ; j < (cv_ptr->image.cols); j++){//j = 0 ; j < cv_ptr->image.cols; j++
            if(Dimage[j] > 0.0){
                Iimage[j] = Dimage[j];//Dimageのj番目の画素にアクセスし、それをIimage[j]に代入 Dimageが既に[mm]になってるっぽい？
//                Ivimage[j] = (char)(255*(Dimage[j]/100000));//距離値を取得できる最大距離で割り、255をかける。255,5.5初期値

                //debug用のdepth値を調べるためのところ。iが縦、jが横。iの最大は479で、jの最大は639。
                if(i==139 && j==200){
//                    ROS_INFO("240行目の320列目のdepth : %f ", Dimage[j]);
                    ROS_INFO("139行目の293列目のdepth : %f ", Dimage[j]);
//                    ROS_INFO("139行目の293列目のdepth[m?] : %d ", Ivimage[j]);
                }else{
                }

                if(Dimage[j] < 450){//ある距離より近くまで、畝が近づいた時 807良さげ
                    sum_i += j;
                    count++;
                }else{
                }
            }else{
            }
        }

        //img3に代入し直し。本来は16UC1なのにcv_bridge::toCvCopyでうまくいかないので。
        for(j = 0 ; j < (cv_ptr->image.cols); j++){//j = 0 ; j < cv_ptr->image.cols; j++
            Simage[j] = (short)Dimage[j];
        }

        center[i] = sum_i / count;//畝の真ん中の画素の位置を出す。
        sum_all_i = sum_all_i + sum_i / count;//全部の行の真ん中の画素を足しあわせる。
    }

    center_center = sum_all_i/image_height;//畝の中心線の重心を出す。

    std_msgs::Float32 center_float32;
    center_float32.data = center_center;
    center_pub.publish(center_float32);


//    ROS_INFO("center_center : %f [個目]", center_center);
//    ROS_INFO("%f", center_center);

//    ave = sum / ((width * 2) * (height * 2));
//    ROS_INFO("depth : %f [m]", ave);



//    cv::Mat img2;//----------CHANGED
//    img2 = cv_ptr2->image.clone();//----------CHANGED
//    img2 = cv_ptr2->image.clone();//----------CHANGED

//    dimg.encoding = sensor_msgs::image_encodings::MONO8;//----------CHANGED
    dimg.encoding = sensor_msgs::image_encodings::MONO16;//----------CHANGED
    dimg.image = img3;//----------CHANGED

    //publish image_d
//  sensor_msgs::ImagePtr image_d_processed = cv_bridge::CvImage(std_msgs::Header(), "mono16", img).toImageMsg();
    //sensor_msgs::ImagePtr image_d_processed = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();// CHANGED from cv_ptr->image

    d_pub.publish(dimg.toImageMsg());//----------CHANGED


//    cv::imshow("DEPTH image", img);
    cv::imshow("DEPTH image cv_ptr->image", cv_ptr->image);
    cv::waitKey(10);
}





int main(int argc, char **argv){
    ros::init(argc, argv, "depth_estimater");

    depth_estimater depth_estimater;

    ros::spin();
    return 0;
}
