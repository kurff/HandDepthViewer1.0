//http://www.cnblogs.com/easymind223


#pragma once
#ifndef _VISUALIZATION_TOOL_H_
#define _VISUALIZATION_TOOL_H_

#include "opencv2/opencv.hpp"

#define HIST_TYPE_MIX 0
#define HIST_TYPE_CONTOUR 1

namespace VisualizationTool
{

//深度显示单通道uchar,float, int类型图像，
void imageSC(std::string windowName, const cv::Mat imgC1);

//以柱状图显示数组，array必须为 CV_32F,CV_32S,CV_8U中的一种，且rows == 1
void ShowArrayHistogram(std::string title, cv::Mat array, cv::Size size = cv::Size(400,400));

//显示一幅图像的直方图，histType为显示方式，HIST_TYPE_MIX表示三通道混合显示，HIST_TYPE_CONTOUR表示以轮廓显示
void showImageHistogram(const std::string windowName, const cv::Mat src, 
    const cv::Mat mask = cv::Mat(), int histType = HIST_TYPE_MIX, 
    cv::Size windowSize = cv::Size(256, 200));

//显示一幅图像的颜色分布图
void showImageColorDistribution(const std::string windowName, const cv::Mat src_3u,
    int nBins = 32, const cv::Mat mask = cv::Mat(), cv::Size windowSize = cv::Size(256, 200));

}
#endif