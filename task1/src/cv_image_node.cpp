#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <stdexcept> // 用于捕获异常

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
    //判断路径是否存在以及该路径表示的否是文件
    if (!fs::exists(dirPath) || !fs::is_directory(dirPath)) {
       ROS_INFO("错误：模板目录不存在或非法 - " );
        return;
    }

    // 遍历目录下的所有PNG文件
    for (const auto& entry : fs::directory_iterator(dirPath))
    {
        //仅遍历.png格式的文件
        if (entry.is_regular_file() && entry.path().extension() == ".png") {
            // 读取灰度图模板
            Mat temp = imread(entry.path().string(), IMREAD_GRAYSCALE);
            if (!temp.empty()) {
                TemplateData td;
                td.image = temp;
                td.label = entry.path().stem().string();  // 文件名为标签
                templates.push_back(td); //将图片数据加载到容器中
               ROS_INFO("已加载模板：");
            }
        }
    }

    if (templates.empty()) {
        ROS_WARN("警告：模板为空！！！");
    }
}

static int iLowH = 0;
static int iHighH = 8;
static int iLows = 50;
static int iHighs = 225;
static int iLowv = 17;
static int iHighv = 225;

static int iLowH2 = 160;
static int iHighH2 = 179;
static int iLows2 = 156;
static int iHighs2 = 225;
static int iLowv2 = 22;
static int iHighv2 = 225;

double minArea = 500;  // 面积小于500的视为噪声，舍去
Size dstSize(200, 200);  // 目标矩形尺寸（与模板匹配）

/*
    打印图片的长，宽，和通道数
    参数：Mat变量
*/
void getChannel(const Mat &image)
{
    //获取图像维度
    int width = image.cols;  // 宽度（列数）
    int height = image.rows;  // 高度（行数）
    int channels = image.channels();  // 通道数

    cout <<"长:"<< width << endl;
    cout <<"宽:"<< height << endl;
    cout <<"通道:"<< channels << endl;
}

/*
    透视变换函数：
    参数：输入原始图像，目标区域角点
    结果：返回矫正后的矩形图像
*/
Mat warpSingleRegion(const Mat& origImg, const vector<Point2f>& srcPts, const Size& dstSize) 
{
    // 定义目标矩形的4个角点（固定顺序）
    vector<Point2f> dstPts = {
        {0, 0},
        {dstSize.width, 0},
        {dstSize.width, dstSize.height},
        {0, dstSize.height}
    };
    
    // 计算透视变换矩阵
    Mat persMat = getPerspectiveTransform(srcPts, dstPts);
    
    // 执行透视变换，得到矫正后的图像
    Mat warpedImg;
    warpPerspective(origImg, warpedImg, persMat, dstSize);
    
    return warpedImg;
}

/* 创建滑杆 */
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
    createTrackbar("HighV2","Threshold2",&iHighv2,225);

}

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
        else if (num >= 25 && num <= 28)           return "B";
        else if (num == 29)           return "O";
        else                          return "X";
    } catch (const exception&) {
        return label; // 非数字标签直接返回
    }
}

/**
 * 统计字符内部的孔洞数量（核心函数）
 * @param charBinary 二值化字符图像（要求：字符为黑色[0]，背景为白色[255]）
 * @return 孔洞数量（-1表示计算失败）
 */
int countCharHoles(const Mat& charBinary) {
    // 1. 基础校验：必须是单通道二值图
    if (charBinary.empty() || charBinary.channels() != 1) {
        ROS_WARN("孔洞统计：输入图像无效（空或非单通道）");
        return -1;
    }

    // 3. 统计“背景区域（白色255）”的连通组件数量
    // 原理：孔洞是“字符内部的背景连通区域”，总背景连通数 = 外部背景（1个） + 孔洞数
    int numComponents;  // 连通组件总数
    Mat labels, stats, centroids;
    numComponents = connectedComponentsWithStats(
        charBinary,        // 输入二值图
        labels,        // 输出：每个像素的连通组件标签
        stats,         // 输出：连通组件的统计信息（面积、位置等）
        centroids,     // 输出：连通组件的中心坐标
        8,             // 连通方式：8邻域（更贴合字符孔洞的连通判断）
        CV_32S         // 标签类型
    );

    // 4. 计算孔洞数：总背景连通数 - 1（减去“图像外部的背景”）
    // 注意：过滤极小的背景连通区域（避免噪声误判为孔洞，如字符边缘毛刺）
    int holeCount = 0;
    const int MIN_HOLE_AREA = 50;  // 最小孔洞面积阈值（根据字符大小调整，200x200字符设50~100）
    for (int i = 1; i < numComponents; i++) {  // i=0是背景（黑色字符），i从1开始是白色区域
        int area = stats.at<int>(i, CC_STAT_AREA);  // 获取第i个连通组件的面积
        if (area > MIN_HOLE_AREA) {
            holeCount++;
        }
    }

    // 5. 最终孔洞数 = 符合面积要求的背景连通数 - 1（外部背景）
    return max(0, holeCount - 1);  // 确保结果非负
}


void imageCallback(const string& imagePath)
{
    //1.创建一个Mat对象，并读取RGB格式图片
    //定义一个opencv图像指针
    /*cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //将ros图像的消息包转换成opencv格式的图片对象
        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
    }//定义一个opencv自己的图像
    Mat image_orign = cv_ptr->image; */

    Mat image_orign = imread(imagePath);
    if(image_orign.empty())
    {
        cout<< "未识别图片"<<endl;
        return;
    }
    
    // 高斯模糊预处理（去噪）
    Mat blurredImg;
    GaussianBlur(image_orign, blurredImg, Size(3, 3), 0);  // 核大小(5,5)，标准差自动计算

    //2.BGR -> HSV,便于红色分割
    Mat hsv_image;
    cvtColor(image_orign,hsv_image,COLOR_BGR2HSV);
    //对V（亮度）进行均衡化
    vector<Mat> hsv_split;
    split(hsv_image,hsv_split);
    equalizeHist(hsv_split[2],hsv_split[2]);
    merge(hsv_split,hsv_image);

    //3. 红色HSV阈值分割，对感兴趣区进行二值化
    Mat mask1, mask2, redMask;
    // 红色区间1
    inRange(hsv_image, Scalar(iLowH, iLows, iLowv), Scalar(iHighH, iHighs, iHighv), mask1);
    //红色区间2
    inRange(hsv_image, Scalar(iLowH2, iLows2, iLowv2), Scalar(iHighH2, iHighs2, iHighv2), mask2);
    redMask = mask1 | mask2;  // 合并两个红色区域掩码

    //开运算
    Mat element = getStructuringElement(MORPH_RECT,Size(3,3));  //3*3的效果比5*5好
    morphologyEx(redMask,redMask,MORPH_OPEN,element);

    //闭运算
    morphologyEx(redMask,redMask,MORPH_CLOSE,element);

    //4. 轮廓检测：寻找红底区域的外部轮廓
    vector<vector<Point>> contours;
    findContours(redMask.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    // 筛选有效轮廓（只保留面积 >= minArea 的轮廓）
    vector<vector<Point>> validContours;  // 存储过滤后的有效轮廓
    for (size_t i = 0; i < contours.size(); i++) 
    {
        double area = contourArea(contours[i]);
        if (area >= minArea) 
        {  // 只保留面积足够大的轮廓
            validContours.push_back(contours[i]);
        }
    }
    if (validContours.empty()) 
    {
        ROS_INFO("警告：未检测到红底区域！");
        return;
    }

    // 5. 多边形逼近：将轮廓简化为四边形（提取4个角点）
    vector<vector<Point2f>> approxContours;  // 存储所有轮廓的逼近结果;
    vector<Point2f> approx;
    for(size_t i = 0;i < validContours.size();i++)
    {   
        approxPolyDP(validContours[i], approx, arcLength(validContours[i], true) * 0.02, true);
        if (approx.size() != 4) 
        {
            continue;
        }
        approxContours.push_back(approx);
    }
    
    //6.透视变换
    vector<Mat> warpedImgs; 
    for(size_t i = 0; i < approxContours.size(); i++) 
    {
        vector<Point2f> srcPts = approxContours[i]; 
        
        // 执行透视变换，获取单张矫正后的图像
        Mat warpedImg = warpSingleRegion(redMask, srcPts, dstSize);
        
        // 用push_back添加到容器（避免越界，自动扩容）
        warpedImgs.push_back(warpedImg);
    }

        // // 模板缩放
        // Mat tempResized;
        // resize(templates[27].image, tempResized, dstSize);

        // // 模板二值化（黑字白底）
        //  Mat tempBinary;
        // threshold(tempResized, tempBinary, 70, 255, THRESH_BINARY_INV);

    //模板匹配并打印标识
    for (size_t i = 0; i < warpedImgs.size(); i++)
    {
        Mat warpedBinarys = warpedImgs[i].clone();
        string recognizedLabel;
        double bestMatchVal = 0.0;
        // 遍历所有模板，找最佳匹配
        for (const auto& temp : templates) 
        {
            // 模板缩放
            Mat tempResized;
            resize(temp.image, tempResized, dstSize);

            // 模板二值化（黑字白底）
            Mat tempBinary;
            threshold(tempResized, tempBinary, 70, 255, THRESH_BINARY_INV);

            // 模板匹配
            Mat matchResult;
            matchTemplate(warpedBinarys, tempBinary, matchResult, TM_CCOEFF_NORMED);
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

        string parsedLabel = parseLabel(recognizedLabel);
         // 输出结果（匹配度过滤）
        if (bestMatchVal > 0.6) 
        {
            
            if(parsedLabel == "5" || parsedLabel == "6" )
            {
                int num = countCharHoles(warpedBinarys);
                ROS_INFO("%d",num);
                if(num == 0) parsedLabel = "5";
                if(num == 1) parsedLabel = "6";
            }
            ROS_INFO("识别结果: %s, 匹配度: %.3f", parsedLabel.c_str(), bestMatchVal);

            // 打印四边形角点坐标
            for (int j = 0; j < 4; j++) 
            {
                Point2f point = approxContours[i][j];
                ROS_INFO("  角点%d: (%.2f, %.2f)", j+1, point.x, point.y);
            }
        }
        else
        {
            continue;
        }
        // 1. 计算轮廓的边界框（外接矩形）
        Rect bbox = boundingRect(validContours[i]); // 获取最小外接矩形（x,y,w,h）

        // 2. 绘制边界框（用绿色框出轮廓）
        rectangle(image_orign, bbox, Scalar(0, 255, 0), 1); // 颜色(B,G,R)=绿色，线宽2

        // 4. 准备标记文本
        string label = parsedLabel;

        // 5. 设置文本位置（在边界框上方，避免遮挡）
        Point textPos(bbox.x, bbox.y - 10); // 左上角x不变，y减10（在框上方）

        // 6. 添加文本标记（用蓝色字体）
        putText(image_orign, label, textPos, 
                FONT_HERSHEY_SIMPLEX, // 字体
                0.4, // 字体大小
                Scalar(0, 0, 255), // 颜色(B,G,R)=蓝色
                1); // 字体厚度
    }

    imshow("bgr",image_orign);
    imshow("hsv",redMask);
   // imshow("re",tempBinary);

    waitKey(1);

}


int main(int argc, char *argv[])
{
     setlocale(LC_ALL,"");
    // 加载模板
    loadTemplates("/home/qq/catkin_ws/src/task1/temple");
    if (templates.empty()) {
        return -1;  // 无模板时退出
    }

    setTrackbar();

   /* //初始化节点
    ros::init(argc,argv,"cv_image_node");
    //创建句柄
    ros::NodeHandle nh;
    //创建订阅者并订阅相机话题
    ros::Subscriber rgb_sub = nh.subscribe("/camera/color/image_raw",1,imageCallback);

    ros::spin();
    */
   namedWindow("bgr");
   namedWindow("hsv");
    //namedWindow("re");

   while (1)
   {
        imageCallback("/home/qq/catkin_ws/src/task1/picture/target1.png");
   }
   
    return 0;
}


/*
void cameraCallback(const sensor_msgs::Image msg)
{
    //定义一个opencv图像指针
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //将ros图像的消息包转换成opencv格式的图片对象
        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
    }

    //定义一个opencv自己的图像
    Mat image_orign = cv_ptr->image; 
    imshow("RGB",image_orign);
    //暂停1ms，让imshow完成显示
    waitKey(1);
}

int main(int argc, char *argv[])
{
    //初始化节点
    ros::init(argc,argv,"cv_image_node");
    //创建句柄
    ros::NodeHandle nh;
    //创建订阅者并订阅相机话题
    ros::Subscriber rgb_sub = nh.subscribe("/camera/color/image_raw",1,cameraCallback);

    //创建一个名为RGB的窗口
    namedWindow("RGB");

    ros::spin();
    return 0;
}*/

