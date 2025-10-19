#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <stdexcept>

namespace fs = std::filesystem;
using namespace cv;
using namespace std;

// 模板数据结构：存储模板图像与标签
struct TemplateData {
    Mat image;       // 模板图像（灰度）
    string label;    // 模板标签
};

vector<TemplateData> templates;  // 存储所有模板

/**
 * 加载模板图像（从指定目录读取所有PNG文件）
 */
void loadTemplates(const string& dirPath) 
{
    if (!fs::exists(dirPath) || !fs::is_directory(dirPath)) {
       ROS_INFO("Error:Templates is empty! - %s", dirPath.c_str()); // 目录路径打印
        return;
    }

    for (const auto& entry : fs::directory_iterator(dirPath))
    {
        if (entry.is_regular_file() && entry.path().extension() == ".png") {
            Mat temp = imread(entry.path().string(), IMREAD_GRAYSCALE);
            if (!temp.empty()) {
                TemplateData td;
                td.image = temp;
                td.label = entry.path().stem().string();  
                templates.push_back(td);
            }
        }
        ROS_INFO("Template successful!"); 
    }

    if (templates.empty()) {
        ROS_WARN(" Templates is empty! ");
    }
}

// HSV阈值参数 
static int iLowH = 0;
static int iHighH = 8;
static int iLows = 50;  //97
static int iHighs = 225;
static int iLowv = 17;
static int iHighv = 225;

static int iLowH2 = 160;
static int iHighH2 = 179;
static int iLows2 = 156;
static int iHighs2 = 225;
static int iLowv2 = 22;
static int iHighv2 = 255;

static int Low = 0;
static int High = 225;

double minArea = 300;  //过滤小于300的轮廓
Size dstSize(200, 200);  // 目标矩形尺寸（与模板匹配）

/**
 * 打印图片的长、宽、通道数
 */
void getChannel(const Mat &image)
{
    int width = image.cols;  
    int height = image.rows;  
    int channels = image.channels();  

    cout <<"长:"<< width << endl;
    cout <<"宽:"<< height << endl;
    cout <<"通道:"<< channels << endl;
}

/**
 * 透视变换函数：矫正目标区域为矩形
 */
Mat warpSingleRegion(const Mat& origImg, const vector<Point2f>& srcPts, const Size& dstSize) 
{
    vector<Point2f> dstPts = {
        {0, 0},
        {(float)dstSize.width, 0},
        {(float)dstSize.width, (float)dstSize.height},
        {0, (float)dstSize.height}
    };
    
    //计算透视变换矩阵
    Mat persMat = getPerspectiveTransform(srcPts, dstPts);
    Mat warpedImg;
    warpPerspective(origImg, warpedImg, persMat, dstSize);
    
    return warpedImg;
}

/**
 * 创建滑杆
 */
void setTrackbar(void)
{
    namedWindow("Threshold1",WINDOW_AUTOSIZE);
    createTrackbar("LowH","Threshold1",&iLowH,179);
    createTrackbar("HighH","Threshold1",&iHighH,179);
    createTrackbar("LowS","Threshold1",&iLows,225);
    createTrackbar("HighS","Threshold1",&iHighs,225);
    createTrackbar("LowV","Threshold1",&iLowv,225);
    createTrackbar("HighV","Threshold1",&iHighv,225);
    
    namedWindow("Threshold2",WINDOW_AUTOSIZE);
    createTrackbar("LowH2","Threshold2",&iLowH2,179);
    createTrackbar("HighH2","Threshold2",&iHighH2,179);
    createTrackbar("LowS2","Threshold2",&iLows2,225);
    createTrackbar("HighS2","Threshold2",&iHighs2,225);
    createTrackbar("LowV2","Threshold2",&iLowv2,225);
    createTrackbar("HighV2","Threshold2",&iHighv2,255);

    namedWindow("Threshold",WINDOW_AUTOSIZE);
    createTrackbar("Low","Threshold",&Low,255);
    createTrackbar("High","Threshold",&High,255);
}

/**
 * 标签解析函数
 */
string parseLabel(const string& label) 
{
    try {
        int num = stoi(label);
        if (num >= 1 && num <= 4)    return "1";
        else if (num >= 5 && num <= 8) return "2";
        else if (num >= 9 && num <= 12) return "3";
        else if (num >= 13 && num <= 16) return "4";
        else if (num >= 17 && num <= 20) return "5";
        else if (num >= 21 && num <= 24) return "6";
        else if (num >= 25 && num <= 28)  return "B";
        else if (num == 29)    return "O";
        else if (num == 30)    return "X";
    } catch (const exception&) {
        return label; 
    }
}

/**
 * 统计字符内部的孔洞数量（核心优化1）
 * 输入：二值化字符图像
 * 输出：孔洞数量（0=5，1=6，2=B）
 */
int countCharHoles(const Mat& charBinary) {
    // 1. 基础校验
    if (charBinary.empty() || charBinary.channels() != 1) {
        ROS_WARN("Picture is error!");
        return -1;
    }

    // 2. 统计背景（白色255）的连通组件（8邻域更贴合字符孔洞判断）
    int numComponents;
    Mat labels, stats, centroids;
    numComponents = connectedComponentsWithStats(
        charBinary,        // 输入二值图
        labels,            // 每个像素的连通组件标签
        stats,             // 连通组件统计信息（面积、位置等）
        centroids,         // 连通组件中心坐标
        4,                 //上下左右联通
        CV_32S             // 标签类型
    );

    // 3. 过滤极小噪声（避免字符边缘毛刺误判为孔洞）
    const int MIN_HOLE_AREA = 40;  // 200x200字符的合理阈值（可微调）
    int validBgCount = 0;
    for (int i = 1; i < numComponents; i++) {  // i=0是黑色字符，i>=1是白色背景/孔洞
        int area = stats.at<int>(i, CC_STAT_AREA);  // 获取连通组件面积
        if (area > MIN_HOLE_AREA) {
            validBgCount++;
        }
    }

    // 4. 计算孔洞数：有效背景连通数 - 1（减去“图像外部的大背景”）
    int holeCount = validBgCount - 1;
    return max(0, holeCount);  // 确保结果非负（无孔洞时返回0）
}

/**
 * 图像回调函数
 */
void imageCallback(const sensor_msgs::Image msg)
{
   cv_bridge::CvImagePtr cv_ptr;
   try
   {
        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
   }
   catch (cv_bridge::Exception& e)
   {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
   }
   Mat image_orign = cv_ptr->image;
   
    // 2. BGR转HSV + V通道均衡化
    Mat hsv_image;
    cvtColor(image_orign,hsv_image,COLOR_BGR2HSV);
    vector<Mat> hsv_split;
    split(hsv_image,hsv_split);
    equalizeHist(hsv_split[2],hsv_split[2]);
    merge(hsv_split,hsv_image);

    // 3. 红色HSV阈值分割 + 形态学操作
    Mat mask1, mask2, redMask;
    inRange(hsv_image, Scalar(iLowH, iLows, iLowv), Scalar(iHighH, iHighs, iHighv), mask1);
    inRange(hsv_image, Scalar(iLowH2, iLows2, iLowv2), Scalar(iHighH2, iHighs2, iHighv2), mask2);
    redMask = mask1 | mask2;

    Mat element = getStructuringElement(MORPH_RECT,Size(3,3));
    morphologyEx(redMask,redMask,MORPH_OPEN,element);  // 开运算去噪
    morphologyEx(redMask,redMask,MORPH_CLOSE,element); // 闭运算补洞

    // 4. 轮廓检测 + 有效轮廓筛选
    vector<vector<Point>> contours;
    findContours(redMask.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    vector<vector<Point>> validContours;  
    for (size_t i = 0; i < contours.size(); i++) 
    {
        double area = contourArea(contours[i]);
        
        // 面积足够 
        if (area >= minArea) 
        {
            validContours.push_back(contours[i]);
        }
    }
    if (validContours.empty()) 
    {
        return;
    }

    // 5. 多边形逼近：提取四边形角点
    vector<vector<Point2f>> approxContours;  
    vector<Point2f> approx;
    for(size_t i = 0;i < validContours.size();i++)
    {   
        approxPolyDP(validContours[i], approx, arcLength(validContours[i], true) * 0.02, true);
        if (approx.size() == 4)  // 只保留四边形（目标为矩形）
        {
            approxContours.push_back(approx);
        }
    }
    if (approxContours.empty())
    {
        return;
    }

    // 6. 透视变换 + 二值化
    vector<Mat> warpedImgs; 
    for(size_t i = 0; i < approxContours.size(); i++) 
    {
        vector<Point2f> srcPts = approxContours[i]; 
        Mat warpedImg = warpSingleRegion(redMask, srcPts, dstSize);
        
        // 二值化（将红mask转为纯黑白，消除灰度波动干扰）
        Mat warpedBinary;
        threshold(warpedImg, warpedBinary, 127, 255, THRESH_BINARY);
        warpedImgs.push_back(warpedBinary);
    }

    // 7. 模板匹配 
    for (size_t i = 0; i < warpedImgs.size(); i++)
    {
        Mat warpedBinary = warpedImgs[i].clone();
        string recognizedLabel;
        double bestMatchVal = 0.0;

        // 7.1 模板匹配
        for (const auto& temp : templates) 
        {
            // 模板缩放（与目标尺寸一致）
            Mat tempResized;
            resize(temp.image, tempResized, dstSize);

            // 模板二值化
            Mat tempBinary;
            threshold(tempResized, tempBinary, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);

            Mat matchResult;
            matchTemplate(warpedBinary, tempBinary, matchResult, TM_CCOEFF_NORMED);
            double minVal, maxVal;
            Point minLoc, maxLoc;
            minMaxLoc(matchResult, &minVal, &maxVal, &minLoc, &maxLoc);

            // 更新最佳匹配
            if (maxVal > bestMatchVal) 
            {
                bestMatchVal = maxVal;
                recognizedLabel = temp.label;
            }
        }

        // 7.2 提取原图字符区域（关键：用于孔洞统计，需原始色彩信息）
        vector<Point2f> srcPts = approxContours[i];
        Mat charFromOrig = warpSingleRegion(image_orign, srcPts, dstSize);  // 从原图透视变换
        Mat charGray, charBinary;
        cvtColor(charFromOrig, charGray, COLOR_BGR2GRAY);  // 转灰度
        // 字符二值化
        threshold(charGray, charBinary, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);

        // 7.3 孔洞统计 
        int holeNum = countCharHoles(charBinary);
        string parsedLabel = parseLabel(recognizedLabel);
        // 根据孔洞数强制修正：5(0孔)、6(1孔)、B(2孔)
        if (parsedLabel == "5" || parsedLabel == "6" || parsedLabel == "B") 
        {
            if (holeNum == 0) 
            {
                parsedLabel = "5";
            } 
            else if (holeNum == 1) 
            {
                parsedLabel = "6";
            } 
            else if (holeNum == 2) 
            {
                parsedLabel = "B";
            }
        }

        // 7.4 结果输出与绘制
        if (bestMatchVal > 0.6) 
        {
            ROS_INFO("result: %s", parsedLabel.c_str());
            // 打印角点坐标
            for (int j = 0; j < 4; j++) 
            {
                Point2f point = approxContours[i][j];
                ROS_INFO("num:%d: (%.2f, %.2f)", j+1, point.x, point.y);
            }
        }

    }
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 1. 加载模板（需确保模板目录路径正确）
    loadTemplates("/opt/ep_ws/src/rmus_solution/task1/temple");
    if (templates.empty()) {
        ROS_ERROR("Template is empty!");
        return -1;
    }

    // 3. 初始化ROS
    ros::init(argc,argv,"cv_image_node");
    ros::NodeHandle nh;
    ros::Subscriber rgb_sub = nh.subscribe("/camera/color/image_raw",1,imageCallback);
    ros::spin();

    return 0;
}
