#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h> 
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>


#include <vector>


typedef enum{
    LIVE_CAMERA,
    IMAGE_MESSAGE
}InputSource;

std::vector<ros::Publisher> pub_moment_invariants;
std::vector<ros::Publisher> pub_moment_invariants_log;

void calcMomentInvariants(const cv::Mat& img, std::vector<double>& results){

    results.clear();
    results.resize(7, 0);

    const cv::Moments moments = cv::moments(img, false);
    cv::HuMoments(moments, results.data());
}

void preprocess(const cv::Mat& raw_img, cv::Mat& preprocessed_img){

    cv::cvtColor(raw_img, preprocessed_img, cv::COLOR_BGR2GRAY);

    const double radius = std::min(preprocessed_img.rows,
                                   preprocessed_img.cols) / 2.0;
    const double radius2 = radius * radius;

    const double cx = preprocessed_img.cols / 2.0;
    const double cy = preprocessed_img.rows / 2.0;

    for(int i_row = 0; i_row < preprocessed_img.rows; i_row++){
        uint8_t* row_head = preprocessed_img.ptr<uint8_t>(i_row);
        for(int i_col = 0; i_col < preprocessed_img.cols; i_col++){
            if((i_col - cx) * (i_col - cx) + (i_row - cy) * (i_row - cy) > radius2){
                *(row_head + i_col) = 0;
            }
        }
    }
}

void publishResults(const std::vector<double>& moment_invariants){

    for(size_t i = 0; i < moment_invariants.size(); i++){
        std_msgs::Float32 tmp_msg1, tmp_msg2;
        tmp_msg1.data = moment_invariants[i];
        tmp_msg2.data = -std::log(std::abs(moment_invariants[i]));
        // if(moment_invariants[i] < 0){
        //     tmp_msg2.data = -tmp_msg2.data;
        // }
        pub_moment_invariants[i].publish(tmp_msg1);
        pub_moment_invariants_log[i].publish(tmp_msg2);
    }
}

void callbackImg(const sensor_msgs::Image& msg){

    cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(msg, "bgr8");

    cv::Mat img_gray;

    preprocess(ptr->image, img_gray);

    std::vector<double> moment_invariants;
    calcMomentInvariants(img_gray, moment_invariants);

    publishResults(moment_invariants);
}

void enterMainLoopLiveCamera(const int camera_device_id){

    cv::VideoCapture cap(camera_device_id);

    if(! cap.isOpened()){
        ROS_ERROR_STREAM("Could not open camera device " << camera_device_id);
        return;
    }

    const std::string window_name = "image";

    cv::namedWindow(window_name, cv::WINDOW_NORMAL);

    cv::Mat img, img_gray;
    while(ros::ok()){
        
        cap >> img;
        preprocess(img, img_gray);
        
        std::vector<double> moment_invariants;
        calcMomentInvariants(img_gray, moment_invariants);

        publishResults(moment_invariants);
        
        cv::imshow(window_name, img_gray);
        const int key = cv::waitKey(1);

        if(key == 'q'){
            break;
        }
        
        ros::spinOnce();
    }
}

void enterMainLoopImageMessage(){
    
    ros::spin();
}

int main(int argc, char** argv){

    ros::init(argc, argv, "test_hu_moment_invariants");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Subscriber sub_img;

    InputSource input_source = IMAGE_MESSAGE;

    int camera_device_id = -1;
    if(pnh.getParam("camera_device_id", camera_device_id)){
        
        ROS_INFO_STREAM("Input: camera device (" << camera_device_id << ")");
        input_source = LIVE_CAMERA;
        
    }else{
        
        sub_img = nh.subscribe("image_raw", 1, callbackImg);
        ROS_INFO_STREAM("Input: image topic (" << sub_img.getTopic() << ")");
        input_source = IMAGE_MESSAGE;
    }

    for(int i = 1; i <= 7; i++){
        const std::string topic1 = std::string("moment_invariant_") + std::to_string(i);
        const std::string topic2 = std::string("moment_invariant_log_") + std::to_string(i);
        pub_moment_invariants.push_back(pnh.advertise<std_msgs::Float32>(topic1, 1));
        pub_moment_invariants_log.push_back(pnh.advertise<std_msgs::Float32>(topic2, 1));
    }

    switch(input_source){
    case LIVE_CAMERA:
        enterMainLoopLiveCamera(camera_device_id);
        break;
    case IMAGE_MESSAGE:
        enterMainLoopImageMessage();
        break;
    default:
        ROS_FATAL("Unexpected input source");
        break;
    }

    return 0;
}

