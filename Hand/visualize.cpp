
#include "visualize.h"

namespace VisualizationTool
{

void imageSC(std::string windowName, const cv::Mat imgC1)
{
    assert(imgC1.channels() == 1 && !imgC1.empty());

    //get min max value of the mat
    double minPixelValue, maxPixelValue;
    cv::minMaxIdx(imgC1, &minPixelValue, &maxPixelValue);
    double valueRange = maxPixelValue - minPixelValue+1;

    //init color table
    const int minSaturation = 20;
    const int colorTableLength = (255 - minSaturation) * 4;    // r -> g -> b
    cv::Scalar colorTable[colorTableLength];

    int i,j;
    for (i = 0, j = minSaturation; i < colorTableLength / 4; i++, j++)
        colorTable[i] = CV_RGB(255, j, minSaturation);
    for (i = colorTableLength / 4, j=1; i < colorTableLength / 2; i++, j++)
        colorTable[i] = CV_RGB(255 - j, 255, minSaturation);
    for (i = colorTableLength/2, j=minSaturation; i < colorTableLength/4*3; i++, j++)
        colorTable[i] = CV_RGB(minSaturation, 255, j);
    for (i = colorTableLength/4*3, j=1; i < colorTableLength; i++, j++)
        colorTable[i] = CV_RGB(minSaturation, 255 - j, 255);


    //draw color table
    const int margin = 20;
    const int tableHeight = 300;;
    const int tableWidth = 150;
    const int barWidth = 30;
    const int barHeight = tableHeight - margin * 2;
    float scale = (float)barHeight / colorTableLength;

    int imageHeight = cv::max(imgC1.rows, tableHeight);
    int imageWidth = imgC1.cols + tableWidth;
    cv::Mat img3u( imageHeight, imageWidth, CV_8UC3, cv::Scalar::all(0));

    for (int i=0; i<barHeight; i++)
    {
        cv::Point pt1(imgC1.cols + margin, margin + i);
        cv::Point pt2(imgC1.cols + margin + barWidth, margin + i);
        cv::line(img3u, pt1, pt2, colorTable[cvRound(i/scale)], 1);
    }

    //illustration
    for (int i=0; i<5; i++)
    {
        float value = minPixelValue + i / 4.0 * valueRange;
        std::stringstream s;
        s<<value;
        int bx = imgC1.cols + margin + barWidth;
        int by = tableHeight - margin - barHeight / 4 * i ;
        cv::line(img3u, cv::Point(bx+5, by), cv::Point(bx+10, by), cvScalarAll(255), 2);
        cv::putText(img3u, s.str(), cv::Point(bx + 20, by + 8),
            CV_FONT_HERSHEY_SIMPLEX, 0.6, cvScalarAll(255), 1);
    }

    //show image
    cv::Mat tim(imgC1.size(), CV_32F);
    imgC1.convertTo(tim, CV_32F);

    for (int y = 0; y < imgC1.rows; y++)
    {
        const float* srcData = tim.ptr<float>(y);
        cv::Vec3b* dstData = img3u.ptr<cv::Vec3b>(y);
        for (int x = 0; x<imgC1.cols; x++)
        {
            double pixel = (srcData[x] - minPixelValue) / valueRange;
            cv::Scalar color = colorTable[cvRound(pixel * (colorTableLength-1))];
            dstData[x] =cv::Vec3b(color.val[2], color.val[1], color.val[0]);
        }
    }
    cv::imshow(windowName, img3u);
}

void ShowArrayHistogram(std::string title, cv::Mat hist, cv::Size size)
{
    CV_Assert(hist.rows == 1);
    cv::Mat imHist = cv::Mat::zeros(size, CV_8UC3);
    int nBins = hist.rows*hist.cols;
    double min, max;
    cv::minMaxLoc(hist, &min, &max);
    double bin_width=(double)size.width/nBins;  
    double bin_unith=(double)size.height/max;

    if(hist.type() == CV_32F)
    {
        float * ptr = hist.ptr<float>(0);
        for(int i=0;i<nBins;i++)  
        {  
            cv::Point p0=cv::Point(i*bin_width,size.height);  
            cv::Point p1=cv::Point((i+1)*bin_width,size.height-ptr[i]*bin_unith);  
            cv::rectangle(imHist, p0, p1, cv::Scalar::all(255), -1, 0, 0);
        } 
    }
    if(hist.type() == CV_32S)
    {
        int* ptr = hist.ptr<int>(0);
        for(int i=0;i<nBins;i++)  
        {  
            cv::Point p0=cv::Point(i*bin_width,size.height);  
            cv::Point p1=cv::Point((i+1)*bin_width,size.height-ptr[i]*bin_unith);  
            cv::rectangle(imHist, p0, p1, cv::Scalar::all(255), -1, 0, 0);
        } 
    }
    if(hist.type() == CV_8U)
    {
        uchar* ptr = hist.ptr<uchar>(0);
        for(int i=0;i<nBins;i++)  
        {  
            cv::Point p0=cv::Point(i*bin_width,size.height);  
            cv::Point p1=cv::Point((i+1)*bin_width,size.height-ptr[i]*bin_unith);  
            cv::rectangle(imHist, p0, p1, cv::Scalar::all(255), -1, 0, 0);
        } 
    }

    cv::namedWindow(title);
    cv::imshow(title, imHist);
}

void showImageHistogram(const std::string windowName, const cv::Mat src,  const cv::Mat mask, int histType, cv::Size windowSize)
{
    CV_Assert(!src.empty());
    if (!mask.empty())
    {
        CV_Assert(mask.type() == CV_8U && src.size() == mask.size());
    }

    cv::Mat src_3u;
    if(src.channels()==1)
        cv::cvtColor(src, src_3u, CV_GRAY2RGB);
    else
        src_3u = src;

    //shrink the src to save time
    float th_maxSide = 300.0;
    int maxSide = cv::max(src_3u.cols , src_3u.rows);
    cv::Mat zoom_3u, zoomMask_1u;

    if (maxSide > th_maxSide)
    {
        float scale = maxSide / th_maxSide;
        zoom_3u.create(src_3u.rows / scale, src_3u.cols / scale, CV_8UC3);
        cv::resize(src_3u, zoom_3u, zoom_3u.size(), 0, 0, cv::INTER_LANCZOS4 );

        if(!mask.empty())
        {
            zoomMask_1u.create(mask.rows / scale, mask.cols / scale, CV_8U);
            cv::resize(mask, zoomMask_1u, zoomMask_1u.size(), 0, 0, cv::INTER_LANCZOS4 );
        }
    }
    else
    {
        zoom_3u = src_3u;
        if(!mask.empty())
            zoomMask_1u = mask;
    }

    std::vector<cv::Mat> rgb_planes;
    cv::split(zoom_3u, rgb_planes );

    int nBins = 255;

    /// 设定取值范围 ( R,G,B) )
    float range[] = { 0, 256 } ;
    const float* histRange = { range };

    bool uniform = true; bool accumulate = false;

    cv::Mat r_hist, g_hist, b_hist;

    /// 计算直方图:
    cv::calcHist( &rgb_planes[0], 1, 0, zoomMask_1u, r_hist, 1, &nBins, &histRange, uniform, accumulate );
    cv::calcHist( &rgb_planes[1], 1, 0, zoomMask_1u, g_hist, 1, &nBins, &histRange, uniform, accumulate );
    cv::calcHist( &rgb_planes[2], 1, 0, zoomMask_1u, b_hist, 1, &nBins, &histRange, uniform, accumulate );

    // 创建直方图画布
    int canvasWidth = windowSize.width; 
    int canvasHeight = windowSize.height;
    int binWidth = cvRound( (double) canvasWidth / nBins );

    cv::Mat histImage(canvasHeight, canvasWidth,  CV_8UC3, cv::Scalar( 0,0,0) );

    /// 将直方图归一化到范围 [ 0, histImage.rows ]
    cv::normalize(r_hist, r_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
    cv::normalize(g_hist, g_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
    cv::normalize(b_hist, b_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

    /// 在直方图画布上画出直方图
    if (histType == HIST_TYPE_CONTOUR)
    {
        for( int i = 1; i < nBins; i++ )
        {
            cv::line( histImage, cv::Point( binWidth*(i-1), canvasHeight - cvRound(r_hist.at<float>(i-1)) ) ,
                cv::Point( binWidth*(i), canvasHeight - cvRound(r_hist.at<float>(i)) ),
                cv::Scalar(255, 0, 0), 2, 8, 0  );
            cv::line( histImage, cv::Point( binWidth*(i-1), canvasHeight - cvRound(g_hist.at<float>(i-1)) ) ,
                cv::Point( binWidth*(i), canvasHeight - cvRound(g_hist.at<float>(i)) ),
                cv::Scalar( 0, 255, 0), 2, 8, 0  );
            cv::line( histImage, cv::Point( binWidth*(i-1), canvasHeight - cvRound(b_hist.at<float>(i-1)) ) ,
                cv::Point( binWidth*(i), canvasHeight - cvRound(b_hist.at<float>(i)) ),
                cv::Scalar( 0, 0, 255), 2, 8, 0  );
        }
    }
    else if (histType == HIST_TYPE_MIX)
    {
        for (int iBin=0; iBin<nBins; iBin++)
        {
            for (int iValue=1; iValue < r_hist.at<float>(iBin); iValue++)
            {
                for (int j=0; j<binWidth; j++)
                {
                    cv::Vec3b& pixel = histImage.at<cv::Vec3b>(canvasHeight - iValue, iBin * binWidth + j);
                    pixel.val[0] = 255;
                }
            }
            for (int iValue=1; iValue < g_hist.at<float>(iBin); iValue++)
            {
                for (int j=0; j<binWidth; j++)
                {
                    cv::Vec3b& pixel = histImage.at<cv::Vec3b>(canvasHeight - iValue, iBin * binWidth + j);
                    pixel.val[1] = 255;
                }
            }
            for (int iValue=1; iValue < b_hist.at<float>(iBin); iValue++)
            {
                for (int j=0; j<binWidth; j++)
                {
                    cv::Vec3b& pixel = histImage.at<cv::Vec3b>(canvasHeight - iValue, iBin * binWidth + j);
                    pixel.val[2] = 255;
                }
            }
        }
    }
    cv::imshow(windowName, histImage );    
}

bool histCompare(std::pair<cv::Scalar,int> v1, std::pair<cv::Scalar,int> v2)
{
    return v1.second < v2.second;
}

int countValueAppearTimes(const cv::Mat srcC1, double value)
{
    CV_Assert(!srcC1.empty() && srcC1.channels()==1);

    cv::Mat r = srcC1 - value;
    int times = cv::countNonZero(r);
    return srcC1.cols * srcC1.rows - times;
}

void showImageColorDistribution(const std::string windowName, const cv::Mat src_3u, int nBins, 
    const cv::Mat mask, cv::Size windowSize)
{
    CV_Assert(!src_3u.empty() );
    if (!mask.empty())
    {
        CV_Assert(mask.type() == CV_8U && src_3u.size() == mask.size());
    }

    //shrink the src to save time
    float th_maxSide = 300.0;
    int maxSide = cv::max(src_3u.cols , src_3u.rows);
    cv::Mat zoom_3u, zoomMask_1u;

    if (maxSide > th_maxSide)
    {
        float scale = maxSide / th_maxSide;
        zoom_3u.create(src_3u.rows / scale, src_3u.cols / scale, CV_8UC3);
        cv::resize(src_3u, zoom_3u, zoom_3u.size(), 0, 0, cv::INTER_LANCZOS4 );

        if(!mask.empty())
        {
            zoomMask_1u.create(mask.rows / scale, mask.cols / scale, CV_8U);
            cv::resize(mask, zoomMask_1u, zoomMask_1u.size(), 0, 0, cv::INTER_LANCZOS4 );
        }
    }
    else
    {
        zoom_3u = src_3u;
        if(!mask.empty())
            zoomMask_1u = mask;
    }
    int maskNonZero = countNonZero(zoomMask_1u);

    //k-means cluster
    cv::Mat clusterMat;
    cv::Mat bestLabels, centers;
    cv::Vec3b* data = zoom_3u.ptr<cv::Vec3b>(0);
    if(mask.empty())
    {
        clusterMat.create(zoom_3u.cols * zoom_3u.rows, 3, CV_32F);
        for (int i=0; i<zoom_3u.cols * zoom_3u.rows; i++)
        {
            cv::Vec3b pixel = data[i];
            clusterMat.at<float>(i, 0) = pixel.val[0];
            clusterMat.at<float>(i, 1) = pixel.val[1];
            clusterMat.at<float>(i, 2) = pixel.val[2];
        }
    }
    else
    {
        clusterMat.create(maskNonZero, 3, CV_32F);
        const uchar* maskData = zoomMask_1u.ptr<uchar>(0);
        for (int i=0, j=0; i<zoomMask_1u.cols * zoomMask_1u.rows; i++)
        {
            if(maskData[i] > 0)
            {
                cv::Vec3b pixel = data[i];
                clusterMat.at<float>(j, 0) = pixel.val[0];
                clusterMat.at<float>(j, 1) = pixel.val[1];
                clusterMat.at<float>(j, 2) = pixel.val[2];
                j++;
            }
        }
    }

    cv::kmeans(clusterMat, nBins, bestLabels, cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0),
        3, cv::KMEANS_PP_CENTERS, centers);

    //statistics
    std::vector<std::pair<cv::Scalar,int>> hist(nBins);
    for (int i=0; i<nBins; i++)
    {
        cv::Scalar color( centers.at<float>(i,0), centers.at<float>(i,1), centers.at<float>(i,2));
        int val = countValueAppearTimes(bestLabels, i);
        hist.at(i) = std::pair<cv::Scalar,int>(color, val);
    }
    std::sort(hist.begin(), hist.end(), histCompare);
    int maxValue = hist[nBins-1].second;

    //canvas
    float scale = (float)windowSize.height / maxValue;
    int binWidth = windowSize.width / nBins;
    cv::Mat canvas(windowSize, CV_8UC3, cv::Scalar::all(30));

    for (int i=0; i<nBins; i++)
    {
        cv::Point pt1(  i    * binWidth, canvas.rows - 1);
        cv::Point pt2( (i+1) * binWidth, canvas.rows - 1 - hist[i].second * scale);        
        cv::rectangle(canvas, pt1, pt2, hist[i].first, -1);
    }
    cv::imshow(windowName, canvas);
}

}