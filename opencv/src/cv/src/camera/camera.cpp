#include "ros/ros.h"
#include <opencv2/opencv.hpp>

using std::endl;
using std::cout;
using namespace cv;


int threshold_value = 128;
cv::Mat frame, gray, binary;

void onTrackbar(int, void*) {
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, binary, threshold_value, 255, cv::THRESH_BINARY);
    cv::imshow("Binary", binary);
}

int main(int argc, char *argv[])
{
    //执行 ros 节点初始化
    ros::init(argc,argv,"camera");
    //创建 ros 节点句柄(非必须)
    ros::NodeHandle n;

    cv::VideoCapture cap(0);
    // 检查摄像头是否成功打开
    if(!cap.isOpened())
    {
        cout << "摄像头打开失败！" << endl;
        return -1;
    }

    cap.set(CAP_PROP_FRAME_WIDTH,640);
    cap.set(CAP_PROP_FRAME_HEIGHT,480);

    // 设置帧率（需摄像头支持）
    cap.set(cv::CAP_PROP_FPS, 60);

    double actual_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double actual_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double actual_fps = cap.get(cv::CAP_PROP_FPS);
    std::cout << "实际分辨率: " << actual_width << "x" << actual_height 
              << ", 实际帧率: " << actual_fps << std::endl;

    while(1)
    {
        cap >> frame; 

        if (frame.empty()) break; // 摄像头断开或数据错误时退出

        onTrackbar(0, nullptr); // 更新显示

        // 步骤3：显示图像
        cv::imshow("Live Camera", frame);
        cv::imshow("Grayscale", gray);
        // 步骤4：检测按键（等待 10ms）
        int key = cv::waitKey(1);
        if (key == 27) break; // ESC 键退出
        std::cout << "FPS: " << cap.get(cv::CAP_PROP_FPS) << std::endl;
    }


    return 0;
}