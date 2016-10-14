//http://www.cnblogs.com/easymind223


#pragma once
#ifndef _VISUALIZATION_TOOL_H_
#define _VISUALIZATION_TOOL_H_

#include "opencv2/opencv.hpp"

#define HIST_TYPE_MIX 0
#define HIST_TYPE_CONTOUR 1

namespace VisualizationTool
{

//�����ʾ��ͨ��uchar,float, int����ͼ��
void imageSC(std::string windowName, const cv::Mat imgC1);

//����״ͼ��ʾ���飬array����Ϊ CV_32F,CV_32S,CV_8U�е�һ�֣���rows == 1
void ShowArrayHistogram(std::string title, cv::Mat array, cv::Size size = cv::Size(400,400));

//��ʾһ��ͼ���ֱ��ͼ��histTypeΪ��ʾ��ʽ��HIST_TYPE_MIX��ʾ��ͨ�������ʾ��HIST_TYPE_CONTOUR��ʾ��������ʾ
void showImageHistogram(const std::string windowName, const cv::Mat src, 
    const cv::Mat mask = cv::Mat(), int histType = HIST_TYPE_MIX, 
    cv::Size windowSize = cv::Size(256, 200));

//��ʾһ��ͼ�����ɫ�ֲ�ͼ
void showImageColorDistribution(const std::string windowName, const cv::Mat src_3u,
    int nBins = 32, const cv::Mat mask = cv::Mat(), cv::Size windowSize = cv::Size(256, 200));

}
#endif