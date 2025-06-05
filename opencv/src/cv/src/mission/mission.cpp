#include "mission.h"
#include <iostream>
using namespace std;
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>
/**
 * @brief 裁剪图像的中心区域
 * @param src 原始图像
 * @param crop_ratio 裁剪比例（0-1之间），控制y方向（垂直）裁剪
 * @param x_ratio 水平裁剪比例（0-1之间），控制x方向（水平）裁剪，值越小裁剪越多
 * @return 裁剪后的图像
 */
static cv::Mat cropCenterRegion(const cv::Mat& src, double crop_ratio = 0.4, double x_ratio = 0.3)
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

// 顺序转换函数
std::vector<cv::Point> convertRectPoints(const std::vector<cv::Point>& points) {
    // 假设输入顺序为：0右下，1左下，2左上，3右上
    if (points.size() != 4) return points; // 非4点直接返回
    std::vector<cv::Point> newPoints(4);
    newPoints[0] = points[2]; // 左上
    newPoints[1] = points[3]; // 右上
    newPoints[2] = points[0]; // 右下
    newPoints[3] = points[1]; // 左下
    return newPoints;
}

// 获取矩形
std::vector<cv::Point> getRect(cv::Mat frame)
{
    // 获取初始矩形四点
    // 这个四个点一般是不变的，所以不需要重复获取。

    // 裁剪中心区域
    cv::Mat cropped = cropCenterRegion(frame);


    // 获取并标记矩形
    // 特别注意的是
    // rectPoints[0]是右下角
    // rectPoints[1]是左下角
    // rectPoints[2]是左上角
    // rectPoints[3]是右上角
    std::vector<cv::Point> rectPoints = getRectAndMark(cropped);

    // 顺序转换
    rectPoints = convertRectPoints(rectPoints);
    // 打印矩形顶点个数，用于调试
    if (!rectPoints.empty()) {
        cv::polylines(cropped, rectPoints, true, cv::Scalar(0, 255, 0), 2);
        cv::imshow("矩形识别结果", cropped);
        cout << "检测到矩形，顶点数: " << rectPoints.size() << endl;
    }
    return rectPoints;
}

// 原有函数：分割两个点之间的线段 - 修改为使用cv::Point
std::vector<cv::Point> divideSegment(const cv::Point& A, const cv::Point& B, int n) {
    std::vector<cv::Point> points;
    if (n <= 0) return points;

    double x1 = A.x, y1 = A.y;
    double x2 = B.x, y2 = B.y;
    
    double dx = (x2 - x1) / n;
    double dy = (y2 - y1) / n;

    for (int i = 0; i <= n; ++i) {
        points.push_back(cv::Point(x1 + i * dx, y1 + i * dy));
    }
    return points;
}

// 新函数：处理矩形的四条边 - 修改为使用cv::Point
std::vector<cv::Point> divideRectangle(const std::vector<cv::Point>& rect, int n) {
    std::vector<cv::Point> allPoints;
    if (rect.size() != 4 || n <= 0) return allPoints;

    // 处理四条边
    for (int i = 0; i < 4; ++i) {
        int next = (i + 1) % 4; // 下一点索引（形成闭环）
        
        // 获取当前边的所有等分点
        std::vector<cv::Point> edgePoints = divideSegment(rect[i], rect[next], n);
        
        // 第一条边添加全部点，后续边跳过起点（避免重复）
        int startIndex = (i == 0) ? 0 : 1;
        
        // 添加点到结果集
        allPoints.insert(allPoints.end(), 
                         edgePoints.begin() + startIndex, 
                         edgePoints.end());
    }
    
    return allPoints;
}

// 校准函数
std::vector<cv::Point> Mission::calibration(cv::Mat frame,HsvThreshold& hsvThresh,cv::VideoCapture& cap)
{
    std::vector<cv::Point> points;
    cout << "开始校准" << endl;
    // 校准上下左右
    int cont = 4;
    // 阻塞试校准
    while(cont)
    {
        // 获取当前图像
        cap >> frame;
        // 裁剪中心区域
        cv::Mat cropped = cropCenterRegion(frame);
        // 显示中心区域
        cv::imshow("中心区域", cropped);

        // 处理并显示HSV阈值结果
        cv::Mat result = thresholdHsv(frame, hsvThresh);
        cv::imshow("HSV阈值处理", result);

        // 应用膨胀腐蚀处理
        result = colorProcessAndDilateErode(result);
        cv::imshow("形态", result);

        // 处理轮廓并在图像上绘制
        std::vector<std::vector<cv::Point>> contours = processContours(result, cropped);
        if(cv::waitKey(1) == 32) // 按下空格键，记录当前激光位置
        {
            // 记录当前激光位置
            points.push_back(contours[0][0]);
            cout << "激光位置: " << contours[0][0] << endl;
            cont--;
        }
    }
    return points;  // 返回屏幕坐标
}

// 第一题回归原点（复位）

void Mission::one(cv::Mat frame,ros::Publisher angle_pub)
{
    // 创建点消息对象
    geometry_msgs::Point point_msg;
    
    // 设置坐标值
    point_msg.x = 500; // 中心点X坐标
    point_msg.y = 500; // 中心点Y坐标
    
    // 发布消息
    angle_pub.publish(point_msg);
    
    std::cout << "已发送回归原点指令" << std::endl;
}

// 第二题 沿着屏幕走
void Mission::two(cv::Mat frame,ros::Publisher angle_pub)
{
    // 遍历四个点
    // 硬编码四个点的坐标
    std::vector<cv::Point> points;
    points.push_back(cv::Point(100, 100));
    points.push_back(cv::Point(900, 100));
    points.push_back(cv::Point(900, 900));
    points.push_back(cv::Point(100, 900));

    // 依次发送四个点
    for (const auto& pt : points) {
        geometry_msgs::Point msg;
        msg.x = pt.x;
        msg.y = pt.y;
        
        // 这里假设有angle_pub发布器可用，如果没有需要传参
        // 由于two()没有参数，这里仅做演示，实际使用时需调整
        // angle_pub.publish(msg);

        // 打印调试信息
        std::cout << "已发送点: (" << pt.x << ", " << pt.y << ")" << std::endl;
        // 可以加延时模拟运动
        ros::Duration(0.5).sleep();
    }
}

// 第三第四题 沿着矩形走
// 因为第三第四题，几乎一模一样，都是围着矩形走
// 而第四题是围绕倾斜的矩形走，比第三题多了一个旋转的步骤
// 所以，第三第四题，可以写成一个函数
void Mission::three(cv::Mat frame,ros::Publisher angle_pub)
{
    // 获取矩形
    std::vector<cv::Point> rectPoints = getRect(frame);

    // 分割矩形
    rectPoints = divideRectangle(rectPoints, 10);

    // 通过迭代器的方式，在图形上绘制园，确保每一个点，以做调试
    while(1)
    {
        for(auto point : rectPoints)
        {
            cv::circle(frame, point, 5, cv::Scalar(0, 0, 255), -1);
        }
        cv::imshow("矩形分割结果", frame);
        cv::waitKey(1);
    }

    std::cout << "mission one" << std::endl;
}



