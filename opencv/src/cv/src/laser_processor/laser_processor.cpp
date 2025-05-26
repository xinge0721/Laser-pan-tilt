#include "laser_processor.h"
#include <iostream>

// 膨胀腐蚀
cv::Mat& colorProcessAndDilateErode(cv::Mat& result)
{
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    cv::dilate(result, result, kernel, cv::Point(-1, -1), 4);

    cv::erode(result, result, kernel, cv::Point(-1, -1), 5);

    cv::dilate(result, result, kernel, cv::Point(-1, -1), 4);

    return result;
}
// 矩形预处理
cv::Mat& rectProcess(cv::Mat& src)
{
    // 保存原始图像用于对比
    cv::Mat original = src.clone();
    cv::imshow("原始二值图", original);
    
    // 使用较小的结构元素进行轻微膨胀，仅连接小的断裂部分
    cv::Mat kernel_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(src, src, kernel_dilate, cv::Point(-1,-1), 1);
    
    // 使用闭操作填充矩形内的小孔洞，但使用较小的结构元素
    cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(src, src, cv::MORPH_CLOSE, kernel_close, cv::Point(-1,-1), 1);
    
    // 应用开操作去除小的噪点
    cv::Mat kernel_open = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(src, src, cv::MORPH_OPEN, kernel_open);
    
    // 使用较小的核进行中值滤波，保留更多细节
    cv::medianBlur(src, src, 3);
    
    // 最后进行轻微的闭操作，确保形状完整但不过度膨胀
    cv::morphologyEx(src, src, cv::MORPH_CLOSE, kernel_close);

    // 显示处理结果
    cv::imshow("形态学处理后", src);
    return src;
}
/**
 * @brief 获取矩形
 * @param src 输入图像
 * @return 矩形的四个顶点
 */ 
std::vector<cv::Point> getRectAndMark(cv::Mat& src)
{
    std::vector<cv::Point> data;
    // 灰度化
    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

    // 二值化
    cv::Mat binary;
    // 使用THRESH_BINARY_INV
    // 使用THRESH_BINARY，会使识别出错，会出现矩形黑色，背景白色，最后框选背景，而不是矩形
    cv::threshold(gray, binary, 127, 255, cv::THRESH_BINARY_INV);
    cv::imshow("灰度二值化", binary);
    rectProcess(binary);
    // 获取轮廓，并获取矩形的四个点
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // 声明在循环外部，用于存储所有检测到的矩形点
    std::vector<cv::Point> box_points;
    
    for(auto &contour : contours)
    {
        // 因为算法库要求，所以需要至少5个点
        // cv::minAreaRect() 的底层实现基于旋转卡壳算法（Rotating Calipers），该算法需要至少 3 个非共线点才能计算最小包围矩形。但在实际应用中：

        // 轮廓点通常由 cv::findContours() 返回，其点集可能存在重复点或近似共线点
        // OpenCV 对输入点集有内部校验机制，若点集无法构成有效凸包（例如所有点共线），函数会抛出异常或返回错误结果
        // 当轮廓点数量 <5 时：

        // 出现所有点共线的概率显著增加（例如 4 个点可能构成一条直线）
        // 无法生成有效的旋转矩形（此时返回的矩形宽高可能为 0）

          if(contour.size() < 5 || cv::contourArea(contour) < 400.0) continue;

        // cv::minAreaRect(contour)
        // 参数：轮廓点集 std::vector<cv::Point>
        // 返回值：cv::RotatedRect 对象，包含中心点、尺寸、旋转角度
        // 风险：当轮廓点少于 5 个时可能计算不准确（需要提前过滤）
          cv::RotatedRect rotated_rect  = cv::minAreaRect(contour);

        // rotated_rect.points(vertices)
        // 作用：将旋转矩形的四个顶点坐标写入 cv::Point2f[4] 数组
        // 注意：输出的顶点是无序的（需要自行排序）
          cv::Point2f vertices[4];
          rotated_rect.points(vertices);

        // 通过快速排序函数，对矩形顶点进行排序
        // 参数一：需要排序的数组开始位置
        // 参数二：需要排序的数组结束位置
        // 参数三：自定义比较函数，按照点的x+y值升序排序（左上角到右下角）
        // 这种排序方式可以将矩形的四个顶点按照左上、右上、左下、右下的顺序排列
        // std::sort(vertices, vertices + 4, [](const cv::Point2f& a, const cv::Point2f& b){
        //     return (a.x + a.y) < (b.x + b.y);
        // });

        // 将浮点坐标转换为整数坐标并存储到box_points中
        // vertices数组中存储的是浮点型坐标(Point2f)，需要转换为整数坐标(Point)才能用于后续处理
        // static_cast<int>进行类型转换，确保坐标值为整数
        // 这里将矩形的四个顶点按顺序添加到box_points向量中，用于后续绘制和处理
        for (int i = 0; i < 4; ++i) {
            box_points.push_back(cv::Point(static_cast<int>(vertices[i].x), static_cast<int>(vertices[i].y)));
        }
    }
    
    // 将box_points的数据添加到data中
    data.insert(data.end(), box_points.begin(), box_points.end());
    return data;
}

/**
 * @brief 处理轮廓并绘制边框
 * @param result 二值化图像
 * @param display 要显示边框的图像
 * @return 处理后的轮廓集合
 */
std::vector<std::vector<cv::Point>> processContours(const cv::Mat& result, cv::Mat& display)
{
    std::vector<std::vector<cv::Point>> filtration;  // 存储轮廓点集
    std::vector<std::vector<cv::Point>> contours;  // 存储过滤后的轮廓点集

    // 边缘检测
    cv::findContours(result, filtration, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // 过滤小轮廓
    for(auto &contour : filtration)
    {
        if (cv::contourArea(contour) > 50.0) {
            contours.push_back(contour);
        }
    }

    // 可以获取的轮廓信息示例：
    // 获取轮廓面积
    // double area = cv::contourArea(contour);
    // // 获取最小外接矩形
    // cv::Rect rect = cv::boundingRect(contour);
    // int x = rect.x;          // 左上角x坐标
    // int y = rect.y;          // 左上角y坐标
    // int width = rect.width;  // 矩形宽度
    // int height = rect.height;// 矩形高度
    // // 获取最小外接旋转矩形
    // cv::RotatedRect rotated_rect = cv::minAreaRect(contour);
    // float angle = rotated_rect.angle; // 旋转角度
    // // 获取中心点坐标
    // cv::Moments m = cv::moments(contour);
    // cv::Point center(m.m10/m.m00, m.m01/m.m00);

    // 绘制边框并打印调试信息
    for(auto contour : contours)
    {
        // 绘制绿色边框矩形（线宽2）
        cv::Rect border_rect = cv::boundingRect(contour);
        cv::rectangle(display, border_rect, cv::Scalar(0, 255, 0), 2);
        
        // 打印调试信息
        std::cout << "轮廓面积: " << cv::contourArea(contour) << std::endl; 
        std::cout << "轮廓点数: " << contour.size() << std::endl;
        std::cout << "轮廓周长: " << cv::arcLength(contour, true) << std::endl;
        std::cout << "轮廓中心: " << cv::moments(contour).m00 << std::endl;
    }
    
    return contours;
} 