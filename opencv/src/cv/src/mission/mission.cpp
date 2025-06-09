#include "mission.h"
#include <iostream>
using namespace std;
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>
#include "./../laser_processor/laser_processor.h"
#include "./../hsv_threshold/hsv_threshold.h"

/*
 * @brief 裁剪图像的中心区域
 * @param src 原始图像
 * @param crop_ratio 裁剪比例（0-1之间），控制y方向（垂直）裁剪
 * @param x_ratio 水平裁剪比例（0-1之间），控制x方向（水平）裁剪，值越小裁剪越多
 * @return 裁剪后的图像
 */
 cv::Mat cropCenterRegion(const cv::Mat& src, double crop_ratio , double x_ratio )
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
// 参数一：HSV阈值
// 参数二：摄像头
std::vector<cv::Point> Mission::calibration(HsvThreshold& hsvThresh,cv::VideoCapture& cap)
{
    cv::Mat frame;
    std::vector<cv::Point> points;
    std::vector<cv::Point> displayPoints; // 用于存储显示用的点（相对于原始图像）

    // 校准上下左右
    int cont = 4;
    // 阻塞试校准
    while(cont)
    {
        // 获取当前图像
        cap >> frame;
        cv::Mat display = frame.clone(); // 用于显示的原始图像副本
        cv::imshow("完整图像", display);
        
        // 裁剪中心区域
        cv::Mat cropped = cropCenterRegion(frame);
        cv::imshow("中心区域", cropped);

        // 处理并显示HSV阈值结果
        cv::Mat result = thresholdHsv(cropped, hsvThresh);
        cv::imshow("HSV阈值处理", result);

        // 应用膨胀腐蚀处理
        result = colorProcessAndDilateErode(result);
        cv::imshow("形态", result);

        // 处理轮廓并在图像上绘制
        std::vector<std::vector<cv::Point>> contours = processContours(result, cropped);
        
        // 显示已确定的校准点
        for(size_t i = 0; i < displayPoints.size(); i++) {
            cv::circle(display, displayPoints[i], 8, cv::Scalar(0, 255, 0), -1);
            cv::putText(display, std::to_string(i+1), displayPoints[i], 
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
        }
        cv::imshow("校准点标注", display);
        
        int key = cv::waitKey(1);
        if(key == 32 && !contours.empty()) // 按下空格键，记录当前激光位置
        {
            // 获取当前裁剪图像中的激光点位置 - 计算轮廓的中心点
            cv::Moments m = cv::moments(contours[0]);
            cv::Point laserPoint(m.m10/m.m00, m.m01/m.m00);
            
            // 计算在原始图像中的位置
            int width = frame.cols;
            int height = frame.rows;
            int crop_width = static_cast<int>(width * X_RATIO);
            int crop_height = static_cast<int>(height * CROP_RATIO);
            int x_offset = (width - crop_width) / 2;
            int y_offset = (height - crop_height) / 2;
            
            // 转换为原始图像坐标
            cv::Point originalPoint(laserPoint.x + x_offset, laserPoint.y + y_offset);
            
            // 记录当前激光位置
            points.push_back(laserPoint); // 记录裁剪图像中的坐标
            displayPoints.push_back(originalPoint); // 记录原始图像中的坐标
            
            // 请遵循以下顺序
            // 左上角
            // 右上角
            // 右下角
            // 左下角
            // 顺序为顺时针，不建议其他顺序
            cout << "激光位置: " << laserPoint << " (裁剪图像坐标)" << endl;
            cout << "原始图像位置: " << originalPoint << " (原始图像坐标)" << endl;
            cout << "已确定校准点 " << (4-cont+1) << "/4" << endl;
            cont--;
            
            // 在当前帧上标注已确定的点
            cv::circle(display, originalPoint, 10, cv::Scalar(0, 0, 255), -1);
            cv::putText(display, std::to_string(4-cont), originalPoint, 
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
            cv::imshow("校准点标注", display);
            cv::waitKey(500); // 显示0.5秒，让用户看清标注
        }
        // 按下ESC键，完全重新开始校准
        else if(key == 27)
        {
            std::cout << "完全重新开始校准" << std::endl;
            cont = 4;
            points.clear();
            displayPoints.clear(); // 清空显示点
            
            // 清除所有窗口上的标记，显示新的干净界面
            cv::Mat cleanFrame;
            cap >> cleanFrame;
            cv::Mat cleanDisplay = cleanFrame.clone();
            cv::imshow("完整图像", cleanDisplay);
            cv::imshow("校准点标注", cleanDisplay);
            
            cv::Mat cleanCropped = cropCenterRegion(cleanFrame);
            cv::imshow("中心区域", cleanCropped);
            
            // 显示重置提示信息
            cv::putText(cleanDisplay, "已重置校准过程，请重新开始", cv::Point(30, 30), 
                      cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            cv::imshow("校准点标注", cleanDisplay);
            cv::waitKey(1000); // 显示1秒提示
        }
    }
    
    // 显示最终的四个校准点
    cv::Mat finalDisplay = frame.clone();
    for(size_t i = 0; i < displayPoints.size(); i++) {
        cv::circle(finalDisplay, displayPoints[i], 10, cv::Scalar(0, 0, 255), -1);
        cv::putText(finalDisplay, std::to_string(i+1), displayPoints[i], 
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
    }
    cv::imshow("最终校准点标注", finalDisplay);
    cv::waitKey(2000); // 显示2秒
    
    return points;  // 返回裁剪图像中的坐标
}

/**
 * @brief 寻找激光点位置
 * @param frame 输入图像
 * @param hsvThresh HSV阈值参数
 * @param show_result 是否显示处理过程和结果
 * @return 激光点位置，如果未检测到返回(-1,-1)
 */
cv::Point findLaserPoint(cv::Mat& cropped, HsvThreshold& hsvThresh, bool show_result = true)
{
    // 裁剪中心区域以便更精确处理
    // cv::Mat cropped = cropCenterRegion(frame);
    
    // 使用HSV阈值处理图像以检测激光点
    cv::Mat result = thresholdHsv(cropped, hsvThresh);
    if (show_result) {
        cv::imshow("HSV处理", result);
    }
    
    // 应用形态学处理增强激光点
    result = colorProcessAndDilateErode(result);
    if (show_result) {
        cv::imshow("形态处理", result);
    }
    
    // 获取激光点轮廓
    std::vector<std::vector<cv::Point>> contours = processContours(result, cropped);
    
    // 如果检测到激光点
    if (!contours.empty()) {
        // 找出面积最大的轮廓
        int maxContourIdx = 0;
        double maxArea = 0;
        const double MIN_AREA = 300.0; // 设置最小面积阈值为300像素
        
        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > maxArea) {
                maxArea = area;
                maxContourIdx = i;
            }
        }
        
        // 检查最大面积是否满足最小阈值要求
        if (maxArea < MIN_AREA) {
            std::cout << "检测到轮廓面积过小: " << maxArea << " 像素，低于阈值 " << MIN_AREA << std::endl;
            return cv::Point(-1, -1);
        }
        
        // 获取最大面积激光点位置
        cv::Moments m = cv::moments(contours[maxContourIdx]);
        cv::Point laserPoint(m.m10/m.m00, m.m01/m.m00);
        
        if (show_result) {
            // 显示结果
            cv::Mat display = cropped.clone();
            // 绘制所有轮廓
            cv::drawContours(display, contours, -1, cv::Scalar(0, 0, 255), 1);
            // 高亮显示最大面积轮廓
            cv::drawContours(display, contours, maxContourIdx, cv::Scalar(0, 255, 0), 2);
            // 标记中心点
            cv::circle(display, laserPoint, 5, cv::Scalar(255, 0, 0), -1);
            cv::putText(display, "Area: " + std::to_string(int(maxArea)), 
                      cv::Point(laserPoint.x + 10, laserPoint.y), 
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            cv::imshow("激光位置", display);
        }
        
        return laserPoint;
    }
    
    // 未检测到激光点
    return cv::Point(-1, -1);
}

/**
 * 判断激光点是否已经接近校准点
 */
#define DISTANCE_THRESHOLD 2.0


// 第一题回归原点（复位）
// 参数一：摄像头
// 参数二：发布者
// 参数三：校准点
// 返回值：中心点坐标
geometry_msgs::Point Mission::one(cv::VideoCapture& cap, ros::Publisher angle_pub, std::vector<cv::Point> & points)
{
    cv::Mat frame;
    // 创建点消息对象
    // 一个发送数据（偏差值），一个是中心点数据
    geometry_msgs::Point point_msg;
    geometry_msgs::Point point_zhon;
    // 设置坐标值
    // 校准点的第一点和第二点的x轴坐标相减，除以2，得出中心点X坐标距离第一点的距离，则再加上第一点的x轴
    // 则是中心点x轴，相对于摄像头的x轴坐标
    // 校准点的第一点和第三点的y轴坐标相减，得出y轴的长度，除以2，得出中心点Y坐标距离第一点的距离，则再加上第一点的y轴
    // 则是中心点y轴，相对于摄像头的y轴坐标
    point_zhon.x = points[0].x + (abs(points[1].x - points[0].x) / 2); // 中心点X坐标
    point_zhon.y = points[0].y + (abs(points[2].y - points[0].y) / 2); // 中心点Y坐标

    
    
    // // 调试用确定的调试点
    // point_zhon.x = points[0].x; // 中心点X坐标
    // point_zhon.y = points[0].y; // 中心点Y坐标


        // 打印调试信息
    std::cout << "中心点X坐标: " << point_msg.x << std::endl;
    std::cout << "中心点Y坐标: " << point_msg.y << std::endl;
    while(ros::ok())
    {
        try
        {
            // 获取当前图像
            cap >> frame;
            // 显示当前图像
            // cv::imshow("当前图像", frame);
            cv::Mat cropped = cropCenterRegion(frame);
            
            // 获取激光位置
            cv::Point laserPoint = findLaserPoint(cropped, hsvThresh);
            
            // 如果检测到激光点
            if (laserPoint.x != -1 && laserPoint.y != -1) {
                // 计算激光点与中心点的偏差
                if(abs(point_msg.x) > DISTANCE_THRESHOLD)
                {
                    point_msg.x = 2;

                }
                else
                {
                    point_msg.x = laserPoint.x - point_zhon.x;
                }
                if(abs(point_msg.y) > DISTANCE_THRESHOLD)
                {
                    point_msg.y = 2;

                }
                else
                {
                     point_msg.y = laserPoint.y - point_zhon.y;

                }
                // // 输出激光位置到控制台
                // std::cout << "激光位置: (" << laserPoint.x << ", " << laserPoint.y << ")" << std::endl;
                // std::cout << "偏差: X=" << point_msg.x << ", Y=" << point_msg.y << std::endl;
                
                // 发布消息
                angle_pub.publish(point_msg);
                
                // 判断激光点是否已经接近中心点
                // const double DISTANCE_THRESHOLD = 5.0;
                
                // 计算激光点与中心点的欧氏距离                
                // 显示距离信息
                std::cout << "x 距离中心点: " << point_msg.x << " 像素" << std::endl;
                std::cout << "y 距离中心点: " << point_msg.y << " 像素" << std::endl;
                
                // 画线（激光和中心点）
                // 将 geometry_msgs::Point 转换为 cv::Point
                cv::Point cvPointZhon(point_zhon.x, point_zhon.y);
                cv::line(cropped, laserPoint, cvPointZhon, cv::Scalar(0, 0, 255), 2);
                cv::imshow("当前图像", cropped);


                // 如果距离小于阈值，说明激光点已经接近目标位置，任务完成
                if (abs(point_msg.x) < DISTANCE_THRESHOLD && abs(point_msg.y) < DISTANCE_THRESHOLD) {
                    std::cout << "激光点已到达目标位置，任务完成！" << std::endl;
                    point_msg.x = 1;
                    point_msg.y = 1;
                    angle_pub.publish(point_msg);
                    // 跳出循环，结束当前任务
                    break;
                }
            }
            
            // 等待10ms
            cv::waitKey(1);

        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    std::cout << "已发送回归原点指令" << std::endl;

    return point_zhon;//返回中心点坐标,重复使用
}

// 第二题 沿着屏幕走
// 参数一：摄像头
// 参数二：发布者
// 参数三：校准点
void Mission::two(cv::VideoCapture& cap, ros::Publisher angle_pub, std::vector<cv::Point> & points)
{
    cv::Mat frame;
    // 创建点消息对象
    geometry_msgs::Point point_msg;
    
    // 直接遍历四个点
    for(int i = 0; i < 4; i++)
    {
        // 直接围绕校准点跑一圈
        while(ros::ok())
        {
            try
            {
                // 获取当前图像
                cap >> frame;
                // 显示当前图像
                cv::imshow("当前图像", frame);

                // 获取激光位置
                cv::Point laserPoint = findLaserPoint(frame, hsvThresh);
                
                // 如果检测到激光点
                if (laserPoint.x != -1 && laserPoint.y != -1) {
                    // 计算激光点与中心点的偏差
                    point_msg.x = laserPoint.x - points[i].x;
                    point_msg.y = laserPoint.y - points[i].y;
                    
                    // 输出激光位置到控制台
                    std::cout << "激光位置: (" << laserPoint.x << ", " << laserPoint.y << ")" << std::endl;
                    std::cout << "偏差: X=" << point_msg.x << ", Y=" << point_msg.y << std::endl;
                    
                    // 发布消息
                    angle_pub.publish(point_msg);
                    
                    // 计算激光点与校准点的欧氏距离
                    double distance = sqrt(point_msg.x * point_msg.x + point_msg.y * point_msg.y);
                    
                    // 显示距离信息
                    std::cout << "距离校准点: " << distance << " 像素" << std::endl;
                    
                    // 添加可视化连线
                    cv::circle(frame, points[i], 5, cv::Scalar(0, 255, 0), -1);  // 绘制目标点
                    cv::line(frame, laserPoint, cv::Point(points[i].x, points[i].y), cv::Scalar(0, 0, 255), 2);  // 绘制连线
                    cv::imshow("当前图像", frame);
                    
                    // 如果距离小于阈值，说明激光点已经接近目标位置，任务完成
                    if (distance < DISTANCE_THRESHOLD) {
                        std::cout << "激光点已到达校准点" << i << "，继续下一个点！" << std::endl;
                        // 跳出循环，结束当前任务
                        break;
                    }
                }
                
                // 等待10ms
                cv::waitKey(10);
                // 等待10ms
                ros::Duration(0.5).sleep();
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }
    }
    
    std::cout << "已完成矩形路径遍历任务" << std::endl;
}

// 第三第四题 沿着矩形走
// 因为第三第四题，几乎一模一样，都是围着矩形走
// 而第四题是围绕倾斜的矩形走，比第三题多了一个旋转的步骤
// 所以，第三第四题，可以写成一个函数
void Mission::three(cv::VideoCapture& cap,ros::Publisher angle_pub)
{
    cv::Mat frame;
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




