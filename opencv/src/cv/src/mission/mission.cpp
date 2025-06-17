#include "mission.h"
#include <iostream>
using namespace std;
#include <opencv2/opencv.hpp>
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
cv::Point findLaserPoint(cv::Mat& cropped, HsvThreshold& hsvThresh, bool show_result = 1)
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
        const double MIN_AREA = 100.0; // 设置最小面积阈值为300像素
        
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
#define DISTANCE_THRESHOLD 4.5

// 移动激光到目标（轮询方式）
// 参数一：摄像头
// 参数二：发布者
// 参数三：目标点
// 返回值：是否到达目标点
// 成功返回true，失败返回false
bool Mission::getPoint(cv::VideoCapture& cap, myserial& serial,const cv::Point & targetPoint)
{
    cv::Mat frame;
    // 创建点消息对象
    cv::Point point_msg;
    
    // 定义静态变量记录连续接近目标点的次数
    static int success_count = 0;
    // 设置需要连续接近的次数阈值
    const int SUCCESS_THRESHOLD = 5;

    // 控制发送频率
    static ros::Time last_send_time = ros::Time(0);
    const ros::Duration send_interval(0.5); // 100ms, 10Hz
    // const ros::Duration send_interval(0.25); // 100ms, 10Hz

    try
    {
        // 获取当前图像
        cap >> frame;
        // 裁剪图像
        cv::Mat cropped = cropCenterRegion(frame);
        
        // 获取激光位置
        cv::Point laserPoint = findLaserPoint(cropped, hsvThresh);
        
        // 如果检测到激光点
        if (laserPoint.x != -1 && laserPoint.y != -1) {
            // 计算激光点与目标点的偏差
            point_msg.x = laserPoint.x - targetPoint.x;
            point_msg.y = laserPoint.y - targetPoint.y;
            
            ros::Time now = ros::Time::now();
            if (now - last_send_time > send_interval)
            {
                // 发布消息
                serial.sendPoint(point_msg.x, point_msg.y);
                last_send_time = now;

                // 显示距离信息
                std::cout << "x 距离目标点: " << point_msg.x << " 像素" << std::endl;
                std::cout << "y 距离目标点: " << point_msg.y << " 像素" << std::endl;
            }
            
            // 画线（激光和目标点）
            cv::line(cropped, laserPoint, targetPoint, cv::Scalar(0, 0, 255), 2);
            cv::circle(cropped, targetPoint, 5, cv::Scalar(0, 255, 0), -1);  // 绘制目标点
            cv::imshow("当前图像", cropped);

            // 如果距离小于阈值，说明激光点接近目标位置
            if (abs(point_msg.x) < DISTANCE_THRESHOLD && abs(point_msg.y) < DISTANCE_THRESHOLD) {
                // 增加连续接近计数
                success_count++;
                std::cout << "接近目标点，连续次数: " << success_count << "/" << SUCCESS_THRESHOLD << std::endl;
                
                // 只有连续多次接近目标点才算真正到达
                if (success_count >= SUCCESS_THRESHOLD) {
                    std::cout << "激光点已稳定到达目标位置，任务完成！" << std::endl;
                    point_msg.x = 0;
                    point_msg.y = 0;
                    serial.sendPoint(point_msg.x, point_msg.y);
                    // 重置计数器，为下一个目标做准备
                    success_count = 0;
                    return true;
                }
            } else {
                // 如果偏离目标点，重置计数器
                if (success_count > 0) {
                    std::cout << "偏离目标点，重置计数器" << std::endl;
                    success_count = 0;
                }
            }
        }
        // 等待1ms以更新界面显示
        cv::waitKey(1);
        return false;  // 未到达目标点，返回false
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        // 出现异常时也重置计数器
        success_count = 0;
        return false;
    }
}


// 第一题回归原点（复位）
// 参数一：摄像头
// 参数二：发布者
// 参数三：校准点
// 返回值：中心点坐标
cv::Point Mission::one(cv::VideoCapture& cap, myserial& serial, std::vector<cv::Point> & points)
{
    cv::Mat frame;

    // 创建点消息对象
    // 一个发送数据（偏差值），一个是中心点数据

    cv::Point point_zhon;

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
    std::cout << "中心点X坐标: " << point_zhon.x << std::endl;
    std::cout << "中心点Y坐标: " << point_zhon.y << std::endl;
    
    // 轮询方式到达中心点
    bool reached = false;
    while(ros::ok() && !reached)
    {
        reached = getPoint(cap, serial, point_zhon);
        // 等待短暂时间以避免CPU占用过高
        cv::waitKey(1);
        ros::spinOnce(); // 允许ROS处理回调
    }

    std::cout << "已成功回归原点" << std::endl;

    return point_zhon;//返回中心点坐标,重复使用
}

// 第二题 沿着屏幕走
// 参数一：摄像头
// 参数二：发布者
// 参数三：校准点
void Mission::two(cv::VideoCapture& cap, myserial& serial, std::vector<cv::Point> & points)
{
    cv::Mat frame;
    // 创建点消息对象
    cv::Point point_msg;
    
    // 直接遍历四个点
    for(int i = 0; i < 4; i++)
    {
        // 轮询方式到达每个目标点
        bool reached = false;
        while(ros::ok() && !reached)
        {
            reached = getPoint(cap, serial, points[i]);
            // 等待短暂时间以避免CPU占用过高
            cv::waitKey(1);
            ros::spinOnce(); // 允许ROS处理回调
        }
        std::cout << "已到达第" << i+1 << "个点" << std::endl;
    }
    
    std::cout << "已完成屏幕走任务" << std::endl;
}

// 第三题 沿着矩形走
// 参数一：摄像头
// 参数二：发布者
void Mission::three(cv::VideoCapture& cap,myserial& serial)
{
    // 1. 获取图像帧
    cv::Mat frame;
    cap >> frame; // 从摄像头获取新的一帧

    if (frame.empty()) {
        std::cerr << "在 Mission::three 中无法从摄像头读取图像帧" << std::endl;
        return;
    }
    
    // 2. 预处理：裁剪图像中心区域
    // 目的是减少背景干扰，聚焦于可能出现矩形的核心区域。
    frame = cropCenterRegion(frame);
    
    // 3. 查找内外轮廓
    // 调用 findInnerAndOuterContours 函数，它会返回一个包含内外旋转矩形顶点的pair。
    auto rect_points_pair = findInnerAndOuterContours(frame);
    std::vector<cv::Point2f> outer_points_f = rect_points_pair.first;
    std::vector<cv::Point2f> inner_points_f = rect_points_pair.second;

    // 4. 绘制与处理轮廓
    // 确保同时找到了外轮廓和内轮廓才继续处理。
    if (!outer_points_f.empty() && !inner_points_f.empty())
    {
        std::cout << "找到内外轮廓" << std::endl;

        // 4.1 将Point2f转换为Point，并组织成轮廓(vector<vector<Point>>)格式
        // 这是因为OpenCV的绘图函数（如drawContours）通常需要vector<Point>格式。
        std::vector<std::vector<cv::Point>> outer_contours;
        // findInnerAndOuterContours返回的顶点是平铺的，每4个点构成一个矩形，需要重新分组。
        for (size_t i = 0; i < outer_points_f.size(); i += 4) {
            std::vector<cv::Point> contour;
            for(size_t j=0; j<4; ++j) {
                if(i+j < outer_points_f.size())
                    contour.push_back(cv::Point(outer_points_f[i+j]));
            }
            outer_contours.push_back(contour);
        }

        std::vector<std::vector<cv::Point>> inner_contours;
        for (size_t i = 0; i < inner_points_f.size(); i += 4) {
            std::vector<cv::Point> contour;
            for(size_t j=0; j<4; ++j) {
                if(i+j < inner_points_f.size())
                    contour.push_back(cv::Point(inner_points_f[i+j]));
            }
            inner_contours.push_back(contour);
        }

        // // 4.2 绘制检测到的轮廓用于可视化
        // // 外轮廓用绿色绘制
        // cv::drawContours(frame, outer_contours, -1, cv::Scalar(0, 255, 0), 2);
        // // 内轮廓用红色绘制
        // cv::drawContours(frame, inner_contours, -1, cv::Scalar(0, 0, 255), 2);

        // // 在轮廓的每个顶点上绘制一个粉色小圆圈，以便更清晰地看到顶点位置。
        // for (const auto& contour : outer_contours)
        // {
        //     for (const auto& point : contour)
        //     {
        //         cv::circle(frame, point, 2, cv::Scalar(255, 0, 255), -1); 
        //     }
        // }
        // for (const auto& contour : inner_contours)
        // {
        //     for (const auto& point : contour)
        //     {
        //         cv::circle(frame, point, 2, cv::Scalar(255, 0, 255), -1); 
        //     }
        // }

        // cv::imshow("内外轮廓识别结果", frame);

        // 5. 计算并规划中间路径
        // 假设我们只处理检测到的第一对内外矩形。
        if (!outer_contours.empty() && !inner_contours.empty() && outer_contours[0].size() == 4 && inner_contours[0].size() == 4)
        {
            std::vector<cv::Point> rectPoints;
            rectPoints.resize(4); // 为4个路径点分配空间
            
            // // 为保证内外顶点能正确配对，先对它们进行排序。
            // // 这里使用一个简单的排序方法：按 (x+y) 坐标和升序排列，
            // // 这通常能将顶点按左上、右上/左下、右下的顺序排列。
            // std::sort(outer_contours[0].begin(), outer_contours[0].end(), [](const cv::Point& a, const cv::Point& b){
            //     return (a.x + a.y) < (b.x + b.y);
            // });
            // std::sort(inner_contours[0].begin(), inner_contours[0].end(), [](const cv::Point& a, const cv::Point& b){
            //     return (a.x + a.y) < (b.x + b.y);
            // });

            // 计算内外矩形对应顶点的中点，作为激光要行走的路径点。
            for(int i = 0; i < 4; i++) {
                rectPoints[i].x = (outer_contours[0][i].x + inner_contours[0][i].x) / 2;
                rectPoints[i].y = (outer_contours[0][i].y + inner_contours[0][i].y) / 2;
            }
            std::vector<cv::Point> dividedPoints = divideRectangle(rectPoints, 3);
            // 7. 执行路径（当前为注释状态）
            // 遍历所有路径点，并控制激光依次到达。
            for(auto& point : dividedPoints)
            {
                // 轮询方式到达每个点
                bool reached = false;
                while(ros::ok() && !reached)
                {
                    reached = getPoint(cap, serial, point);    
                    // 等待短暂时间以避免CPU占用过高
                    ros::spinOnce(); // 允许ROS处理回调
                }
            }
            std::cout << "已完成矩形路径遍历任务" << std::endl;
            cv::waitKey(1);
        }
        else
        {
            std::cout << "轮廓顶点数不为4，无法计算路径" << std::endl;
        }
    }
    else 
    {
        std::cout << "未找到足够的内外轮廓" << std::endl;
    }

    cv::imshow("轮廓", frame);
    cv::waitKey(1);
}

// 第四题 沿着斜着的矩形走
// 参数一：摄像头
// 参数二：发布者
void Mission::four(cv::VideoCapture& cap,myserial& serial)
{
    cv::Mat frame;
    cap >> frame;
    // 获取矩形
    std::vector<cv::Point> rectPoints = getRect(frame);

    // 分割矩形
    rectPoints = divideRectangle(rectPoints, 10);

    // 遍历矩形上的所有点
    for(auto& point : rectPoints)
    {
        // 轮询方式到达每个点
        bool reached = false;
        while(ros::ok() && !reached)
        {
            reached = getPoint(cap, serial, point);    
            // 等待短暂时间以避免CPU占用过高
            ros::spinOnce(); // 允许ROS处理回调
        }
    }
    std::cout << "已完成斜矩形路径遍历任务" << std::endl;
}

// 第五题 激光跟踪
// 参数一：摄像头
// 参数二：发布者   
void Mission::five(cv::VideoCapture& cap,myserial& serial)
{
    cv::Mat frame;

    // 创建一个定时两秒的循环
    ros::Time start_time = ros::Time::now();
    ros::Duration loop_duration(2.0); // 2秒的持续时间
    bool reached = false;
    while (ros::ok())
    {
        // 获取当前图像
        cap >> frame;
        cv::Mat cropped = cropCenterRegion(frame);
        
        // 获取当前时间
        ros::Time current_time = ros::Time::now();
        
        // 计算已经过去的时间
        ros::Duration elapsed_time = current_time - start_time;
        
        // 获取绿色激光位置
        cv::Point greenLaserPoint = findLaserPoint(cropped, GREEN_HSV);

        // 如果检测到激光点
        if (greenLaserPoint.x != -1 && greenLaserPoint.y != -1) {
            std::cout << "激光点位置: " << greenLaserPoint << std::endl;
            
            // 轮询方式到达目标点，但在这里我们只尝试一次，不等待成功
            // 因为目标点可能在不断移动
            reached = getPoint(cap, serial, greenLaserPoint);

        }

        // 如果已经过去了2秒，则说明完成任务，退出循环
        if (elapsed_time >= loop_duration)
        {
            std::cout << "两秒时间到" << std::endl;
            std::cout << "激光跟踪任务完成" << std::endl;
            if(reached) {
                std::cout << "已成功跟踪到激光点" << std::endl;
            }
            else{
                std::cout << "激光点未到达目标位置，任务失败" << std::endl;
            }
            return;
        }
        
        // 处理ROS回调
        ros::spinOnce();
    }
}

void Mission::six(cv::VideoCapture& cap, myserial& serial)
{
    std::cout << "第六题未实现" << std::endl;
}


// 测试函数 - 计算两次激光位置的差值
void Mission::testLaserDifference(cv::VideoCapture& cap) 
{
    cv::Mat frame;
    cv::Point firstPosition(-1, -1);  // 第一次激光位置
    cv::Point secondPosition(-1, -1);  // 第二次激光位置
    bool firstRecorded = false;  // 是否已记录第一个位置
    
    std::cout << "按空格键记录第一个激光位置..." << std::endl;
    
    while(ros::ok()) {
        try {
            // 获取当前图像
            cap >> frame;
            if(frame.empty()) {
                std::cerr << "获取图像失败" << std::endl;
                continue;
            }
            
            // 裁剪中心区域
            cv::Mat cropped = cropCenterRegion(frame);
            
            // 获取激光位置
            cv::Point laserPoint = findLaserPoint(cropped, hsvThresh);
            
            // 在图像上显示当前激光点
            if (laserPoint.x != -1 && laserPoint.y != -1) {
                cv::circle(cropped, laserPoint, 5, cv::Scalar(0, 255, 0), -1);
                cv::putText(cropped, "当前位置: (" + std::to_string(laserPoint.x) + 
                          ", " + std::to_string(laserPoint.y) + ")", 
                          cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 
                          0.5, cv::Scalar(0, 255, 0), 1);
                
                // 如果已记录第一个位置，显示连线
                if (firstRecorded) {
                    cv::circle(cropped, firstPosition, 5, cv::Scalar(0, 0, 255), -1);
                    cv::line(cropped, firstPosition, laserPoint, 
                            cv::Scalar(0, 0, 255), 2);
                }
            }
            
            // 显示提示信息
            std::string prompt = firstRecorded ? 
                "按空格键记录第二个位置并计算差值..." : 
                "按空格键记录第一个位置...";
            cv::putText(cropped, prompt, cv::Point(10, 60), 
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
            
            cv::imshow("激光位置测试", cropped);
            
            int key = cv::waitKey(1);
            if (key == 27) { // ESC键退出
                break;
            } else if (key == 32 && laserPoint.x != -1 && laserPoint.y != -1) { // 空格键记录位置
                if (!firstRecorded) {
                    // 记录第一个位置
                    firstPosition = laserPoint;
                    firstRecorded = true;
                    std::cout << "第一个激光位置已记录: (" << firstPosition.x 
                            << ", " << firstPosition.y << ")" << std::endl;
                    std::cout << "按空格键记录第二个位置并计算差值..." << std::endl;
                } else {
                    // 记录第二个位置并计算差值
                    secondPosition = laserPoint;
                    std::cout << "第二个激光位置已记录: (" << secondPosition.x 
                            << ", " << secondPosition.y << ")" << std::endl;
                    
                    // 计算差值
                    int xDiff = secondPosition.x - firstPosition.x;
                    int yDiff = secondPosition.y - firstPosition.y;
                    double distance = sqrt(xDiff * xDiff + yDiff * yDiff);
                    
                    std::cout << "======================" << std::endl;
                    std::cout << "差值结果:" << std::endl;
                    std::cout << "X轴差值: " << xDiff << " 像素" << std::endl;
                    std::cout << "Y轴差值: " << yDiff << " 像素" << std::endl;
                    std::cout << "两点距离: " << distance << " 像素" << std::endl;
                    std::cout << "======================" << std::endl;
                    
                    // 在图像上显示差值信息
                    cv::Mat resultImage = cropped.clone();
                    cv::circle(resultImage, firstPosition, 5, cv::Scalar(0, 0, 255), -1);
                    cv::circle(resultImage, secondPosition, 5, cv::Scalar(0, 255, 0), -1);
                    cv::line(resultImage, firstPosition, secondPosition, 
                            cv::Scalar(255, 0, 0), 2);
                    
                    cv::putText(resultImage, "起点: (" + std::to_string(firstPosition.x) + 
                              ", " + std::to_string(firstPosition.y) + ")", 
                              cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 
                              0.5, cv::Scalar(0, 0, 255), 1);
                    cv::putText(resultImage, "终点: (" + std::to_string(secondPosition.x) + 
                              ", " + std::to_string(secondPosition.y) + ")", 
                              cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 
                              0.5, cv::Scalar(0, 255, 0), 1);
                    cv::putText(resultImage, "X轴差值: " + std::to_string(xDiff) + " 像素", 
                              cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 
                              0.5, cv::Scalar(255, 0, 0), 1);
                    cv::putText(resultImage, "Y轴差值: " + std::to_string(yDiff) + " 像素", 
                              cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 
                              0.5, cv::Scalar(255, 0, 0), 1);
                    cv::putText(resultImage, "距离: " + std::to_string(distance) + " 像素", 
                              cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 
                              0.5, cv::Scalar(255, 0, 0), 1);
                    
                    cv::imshow("位置差值结果", resultImage);
                    cv::waitKey(0);  // 等待按键继续
                    
                    // 重置记录状态，准备下一轮测试
                    firstRecorded = false;
                    firstPosition = cv::Point(-1, -1);
                    secondPosition = cv::Point(-1, -1);
                    std::cout << "按空格键记录第一个位置..." << std::endl;
                }
            }
            
            // 处理ROS回调
            ros::spinOnce();
        } catch (const std::exception& e) {
            std::cerr << "错误: " << e.what() << std::endl;
        }
    }
    
    cv::destroyAllWindows();
}
