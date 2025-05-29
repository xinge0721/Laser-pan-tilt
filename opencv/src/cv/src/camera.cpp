#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include "./hsv_threshold/hsv_threshold.h"
#include "./laser_processor/laser_processor.h"
#include <vector>
#include <std_srvs/Trigger.h>
#include "Serial/date.h"

// 自定义服务消息类型的包含路径
// 实际使用时，需要创建以下服务文件并在CMakeLists.txt中配置
// 这里使用注释表示，实际应替换为正确的包含路径
// #include "your_package/ModeService.h"

// 存储角度数据的全局变量
int x_angle = 500;
int y_angle = 500;
bool has_data = false;
int current_mode = 1; // 当前工作模式

/**
 * @brief 角度服务回调函数
 * 当客户端请求角度数据时，根据当前模式返回对应处理的角度数据
 * @param req 服务请求 
 * @param res 服务响应，包含角度数据字符串
 * @return 服务处理结果
 * 
 * 注意：由于标准的Trigger服务不支持在请求中传递参数，
 * 所以这里我们使用全局变量current_mode来表示当前模式
 */
bool angleServiceCallback(Serial::date::Request &req, Serial::date::Response &res)
{
    // ROS_INFO("收到服务请求，当前模式: %d", req.mood);
    std::cout <<"收到服务请求，当前模式: " << req.mood << std::endl;
    switch (req.mood)
    {
    case 1:
        /* code */
        break;
    case 2:
        /* code */
        break;    
    case 3:
        /* code */
        break;
    default:
        break;
    }
}

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
cv::Mat cropCenterRegion(const cv::Mat& src, double crop_ratio = 0.4, double x_ratio = 0.3)
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

/**
 * @brief 根据指定模式处理角度数据
 * @param mode 处理模式
 * 
 * 不同模式下对检测到的角度数据进行不同处理
 */
void processModeAngleData(int mode)
{
    // 更新当前模式
    current_mode = mode;
    
    // 根据当前模式调整处理方式
    switch(current_mode) {
        case 1:
            // 模式1：标准处理
            // 不做特殊处理，使用原始检测数据
            break;
        case 2:
            // 模式2：精确模式
            // 这里可以添加平滑处理或精确定位算法
            break;
        case 3:
            // 模式3：快速模式
            // 这里可以添加快速但精度可能较低的处理
            break;
        default:
            // 默认模式
            break;
    }
}

/**
 * @brief 更新当前工作模式
 * 
 * 这个函数可以从ROS参数服务器、订阅话题或者其他来源获取当前模式
 * 在实际应用中，可以根据需要修改模式获取方式
 */
void updateCurrentMode()
{
    // 示例：每30秒自动切换一次模式，从1到3循环
    static ros::Time last_change = ros::Time::now();
    ros::Time now = ros::Time::now();
    
    if ((now - last_change).toSec() > 30.0) {
        current_mode = (current_mode % 3) + 1;
        last_change = now;
        ROS_INFO("摄像头模式自动切换为: %d", current_mode);
    }
    
    // 也可以从ROS参数服务器获取模式
    // int mode;
    // if (ros::param::get("camera_mode", mode)) {
    //     if (mode != current_mode) {
    //         current_mode = mode;
    //         ROS_INFO("从参数服务器获取新模式: %d", current_mode);
    //     }
    // }
}

int main(int argc, char *argv[])
{
    //执行 ros 节点初始化
    ros::init(argc,argv,"camera");
    //创建 ros 节点句柄(非必须)
    ros::NodeHandle n;
    
    // 创建角度服务服务器
    ros::ServiceServer angle_service = n.advertiseService("angle_service", angleServiceCallback);
    
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
    while(ros::ok())
    {
        // 更新当前工作模式
        updateCurrentMode();
        
        cap >> frame; // 从摄像头获取原始图片

        // 裁剪中心区域
        cv::Mat cropped = cropCenterRegion(frame);
        cv::Mat croppeds = cropped;
        // 获取并标记矩形
        std::vector<cv::Point> rectPoints = getRectAndMark(croppeds);
        // 打印矩形顶点个数，用于调试
        if (!rectPoints.empty()) {
            cv::polylines(croppeds, rectPoints, true, cv::Scalar(0, 255, 0), 2);
            cv::imshow("矩形识别结果", croppeds);
            std::cout << "检测到矩形，顶点数: " << rectPoints.size() << std::endl;
        }
        // 处理并显示HSV阈值结果
        cv::Mat result = thresholdHsv(cropped, hsvThresh);
        cv::imshow("HSV阈值处理", result);

        // 应用膨胀腐蚀处理
        result = colorProcessAndDilateErode(result);
        cv::imshow("形态", result);

        // 处理轮廓并在图像上绘制
        std::vector<std::vector<cv::Point>> contours = processContours(result, cropped);
        // 检查轮廓和矩形点是否为空
        if (!contours.empty() && !rectPoints.empty()) {
            cv::line(cropped, rectPoints[2], contours[0][0], 
                cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
                
            // 更新角度数据，使用检测到的点坐标
            x_angle = contours[0][0].x;
            y_angle = contours[0][0].y;
            has_data = true;
            
            // 根据当前模式处理角度数据
            processModeAngleData(current_mode);
            
            // 显示当前模式和角度数据
            std::string mode_text = "模式: " + std::to_string(current_mode);
            cv::putText(cropped, mode_text, cv::Point(10, 20), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            
            std::string angle_text = "X: " + std::to_string(x_angle) + " Y: " + std::to_string(y_angle);
            cv::putText(cropped, angle_text, cv::Point(10, 40), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }
        
        cv::imshow("中心区域", cropped);
        if(cv::waitKey(1) == 27) // ESC键退出
            break;
        
        // 处理ROS服务回调
        ros::spinOnce();
    }
    return 0;
}