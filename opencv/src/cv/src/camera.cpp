#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include "./hsv_threshold/hsv_threshold.h"
#include "./laser_processor/laser_processor.h"
#include <vector>
/**
 * @brief 初始化摄像头
 * @param camera_id 摄像头ID
 * @param width 图像宽度
 * @param height 图像高度
 * @param fps 帧率
 * @return 初始化好的VideoCapture对象
 */
cv::VideoCapture initCamera(int camera_id, int width, int height, int fps)
{
    cv::VideoCapture cap(camera_id);
    // 检查摄像头是否成功打开
    if(!cap.isOpened())
    {
        std::cout << "摄像头打开失败！" << std::endl;
        return cap;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_FPS, fps);
    
    return cap;
}

/**
 * @brief 裁剪图像的中心区域
 * @param src 原始图像
 * @param crop_ratio 裁剪比例（0-1之间），控制y方向（垂直）裁剪
 * @param x_ratio 水平裁剪比例（0-1之间），控制x方向（水平）裁剪，值越小裁剪越多
 * @return 裁剪后的图像
 */
cv::Mat cropCenterRegion(const cv::Mat& src, double crop_ratio = 0.45, double x_ratio = 0.4)
{
    // 获取图像尺寸
    int width = src.cols;
    int height = src.rows;
    // 计算中心区域边界，x方向使用更小的比例
    int crop_width = static_cast<int>(width * x_ratio);  // x方向收紧
    int crop_height = static_cast<int>(height * crop_ratio);
    int x = (width - crop_width) / 2;
    int y = (height - crop_height) / 2;
    // 裁剪并返回中心区域
    return src(cv::Rect(x, y, crop_width, crop_height));
}

int main(int argc, char *argv[])
{
    //执行 ros 节点初始化
    ros::init(argc,argv,"camera");
    //创建 ros 节点句柄(非必须)
    ros::NodeHandle n;

    // 初始化摄像头
    auto cap = initCamera(0,640,480,60);
    if (!cap.isOpened()) {
        std::cerr << "摄像头打开失败！" << std::endl;
        return -1;
    }

    // 使用红色HSV预设
    HsvThreshold hsvThresh = RED_HSV;
    
    // 创建HSV阈值调节滑块
    createHsvSliders("红色HSV阈值", hsvThresh);

    cv::Mat frame;
    while(1)
    {
        cap >> frame; // 从摄像头获取原始图片

        // 裁剪中心区域
        cv::Mat cropped = cropCenterRegion(frame);
        
        // 获取并标记矩形
        std::vector<cv::Point> rectPoints = getRectAndMark(cropped);
        // 打印矩形顶点个数，用于调试
        if (!rectPoints.empty()) {
            std::cout << "检测到矩形，顶点数: " << rectPoints.size() << std::endl;
        }

        // 处理并显示HSV阈值结果
        cv::Mat result = thresholdHsv(cropped, hsvThresh);
        cv::imshow("HSV阈值处理", result);

        // 应用膨胀腐蚀处理
        result = colorProcessAndDilateErode(result);
        cv::imshow("形态", result);

        // 处理轮廓并在图像上绘制
        processContours(result, cropped);
        cv::imshow("中心区域", cropped);
        if(cv::waitKey(1) == 27) // ESC键退出
            break;
    }
    return 0;
}