#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include "./hsv_threshold/hsv_threshold.h"
#include "./laser_processor/laser_processor.h"
#include <vector>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include "./mission/mission.h"

// 自定义服务消息类型的包含路径
// 实际使用时，需要创建以下服务文件并在CMakeLists.txt中配置
// 这里使用注释表示，实际应替换为正确的包含路径
// #include "your_package/ModeService.h"

// 存储角度数据的全局变量
int x_angle = 500;
int y_angle = 500;
bool has_data = false;
int current_mode = 1; // 当前工作模式
// 使用红色HSV预设
HsvThreshold hsvThresh = RED_HSV;

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
    cv::VideoCapture cap(camera_id, cv::CAP_V4L2); // 显式使用V4L2后端
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

// 矩形处理函数
// 将剧情的四个点之间，切成无数个小点。
int main(int argc, char *argv[])
{
    //执行 ros 节点初始化
    ros::init(argc,argv,"camera");
    //创建 ros 节点句柄(非必须)
    ros::NodeHandle n;
    
    // 创建角度发布者（替代原来的服务）
    ros::Publisher angle_pub = n.advertise<geometry_msgs::Point>("angle_data", 10);
    // 创建模式发布者
    ros::Publisher mode_pub = n.advertise<std_msgs::Int32>("camera_mode", 10);
    
    // 初始化摄像头
    auto cap = initCamera(0,640,480,60);
    if (!cap.isOpened()) {
        std::cerr << "摄像头打开失败！" << std::endl;
        return -1;
    }
    else
        std::cout << "摄像头打开成功"<<std::endl;

    // 创建Mission对象
    Mission mission;

    // 创建HSV阈值调节滑块
    createHsvSliders("红色HSV阈值", hsvThresh);

    cv::Mat frame;

    cap >> frame; // 从摄像头获取原始图片
    // 校准（必须的）
    std::cout << "开始校准"<<std::endl;
    std::vector<cv::Point> rectPoints = mission.calibration(hsvThresh,cap);
    
    cap >> frame;
    frame = cropCenterRegion(frame);

    // 校准顺序为左上角，然后顺时针
    for(auto point : rectPoints)
    {
        std::cout << "矩形顶点: " << point << std::endl;
        // 画个圆，将标注的点画出来
        cv::circle(frame, point, 10, cv::Scalar(0, 0, 255), -1);
    }

    cv::imshow("校准结果", frame);

    sleep(2);
    cv::destroyAllWindows();  // 关闭所有OpenCV创建的窗口
    while(ros::ok())
    {
        mission.one(cap,angle_pub,rectPoints);

        // sleep(50);
        cv::destroyAllWindows();  // 关闭所有OpenCV创建的窗口

        // cv::destroyAllWindows();  // 关闭所有OpenCV创建的窗口
        // mission.two(cap,angle_pub,rectPoints);
        // 处理ROS回调
        ros::spinOnce();
    }
    return 0;
}

// 放弃的代码，保留下来，方便以后参考
// int main(int argc, char *argv[])
// {
//     //执行 ros 节点初始化
//     ros::init(argc,argv,"camera");
//     //创建 ros 节点句柄(非必须)
//     ros::NodeHandle n;
    
//     // 创建角度发布者（替代原来的服务）
//     ros::Publisher angle_pub = n.advertise<geometry_msgs::Point>("angle_data", 10);
//     // 创建模式发布者
//     ros::Publisher mode_pub = n.advertise<std_msgs::Int32>("camera_mode", 10);
    
//     // 初始化摄像头
//     auto cap = initCamera(0,640,480,60);
//     if (!cap.isOpened()) {
//         std::cerr << "摄像头打开失败！" << std::endl;
//         return -1;
//     }

//     // 创建Mission对象
//     Mission mission;

//     // 使用红色HSV预设
//     HsvThreshold hsvThresh = RED_HSV;
    
//     // 创建HSV阈值调节滑块
//     // createHsvSliders("红色HSV阈值", hsvThresh);

//     cv::Mat frame;

//     cap >> frame; // 从摄像头获取原始图片
//     // 校准（必须的）
//     std::vector<cv::Point> rectPoints = mission.calibration(frame,hsvThresh,cap);

//     // 校准顺序为左上角，然后顺时针
//     for(auto point : rectPoints)
//     {
//         std::cout << "矩形顶点: " << point << std::endl;
//         // 画个圆，将标注的点画出来
//         cv::circle(frame, point, 10, cv::Scalar(0, 0, 255), -1);
//     }
//     cv::imshow("校准结果", frame);
//     while(1)
//     {
//         // 什么都不做
//         // 确认校准
//         if(cv::waitKey(1) == 32)
//         {
//             break;
//         }
//     }
    
//     while(ros::ok())
//     {
//         // 更新当前工作模式
//         updateCurrentMode();
        
//         cap >> frame; // 从摄像头获取原始图片

//         // 裁剪中心区域
//         cv::Mat cropped = cropCenterRegion(frame);
//         cv::Mat croppeds = cropped;
//         // 获取并标记矩形
//         std::vector<cv::Point> rectPoints = getRectAndMark(croppeds);

//         // 打印矩形顶点个数，用于调试
//         if (!rectPoints.empty()) {
//             cv::polylines(croppeds, rectPoints, true, cv::Scalar(0, 255, 0), 2);
//             cv::imshow("矩形识别结果", croppeds);
//             std::cout << "检测到矩形，顶点数: " << rectPoints.size() << std::endl;
//         }

//         // 处理并显示HSV阈值结果
//         cv::Mat result = thresholdHsv(cropped, hsvThresh);
//         cv::imshow("HSV阈值处理", result);

//         // 应用膨胀腐蚀处理
//         result = colorProcessAndDilateErode(result);
//         cv::imshow("形态", result);

//         // 处理轮廓并在图像上绘制
//         std::vector<std::vector<cv::Point>> contours = processContours(result, cropped);
//         // 检查轮廓和矩形点是否为空
//         if (!contours.empty() && !rectPoints.empty()) {
//             cv::line(cropped, rectPoints[2], contours[0][0], 
//                 cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
                
//             // 更新角度数据，使用检测到的点坐标
//             x_angle = contours[0][0].x;
//             y_angle = contours[0][0].y;
//             has_data = true;
            
//             // 根据当前模式处理角度数据
//             processModeAngleData(current_mode);
            
//             // 显示当前模式和角度数据
//             std::string mode_text = "模式: " + std::to_string(current_mode);
//             cv::putText(cropped, mode_text, cv::Point(10, 20), 
//                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            
//             std::string angle_text = "X: " + std::to_string(x_angle) + " Y: " + std::to_string(y_angle);
//             cv::putText(cropped, angle_text, cv::Point(10, 40), 
//                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
                       
//             // 发布角度数据
//             geometry_msgs::Point angle_msg;
//             angle_msg.x = x_angle;
//             angle_msg.y = y_angle;
//             angle_pub.publish(angle_msg);
            
//             // 发布当前模式
//             std_msgs::Int32 mode_msg;
//             mode_msg.data = current_mode;
//             mode_pub.publish(mode_msg);
//         }
        
//         cv::imshow("中心区域", cropped);
//         if(cv::waitKey(1) == 27) // ESC键退出
//             break;
        
//         // 处理ROS回调
//         ros::spinOnce();
//     }
//     return 0;
// }