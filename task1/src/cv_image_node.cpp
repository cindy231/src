#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <filesystem>
#include <vector>
#include <cmath>
#include <map>
#include <algorithm>
#include <stdexcept>
#include <image_msgs/FaceInfo.h>

namespace fs = std::filesystem;
using namespace cv;
using namespace std;

// HSV阈值参数 
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
static int iHighv2 = 255;

static int Low = 0;
static int High = 225;

// 创建滑杆（HSV阈值调整）
void setTrackbar(void) 
{
    namedWindow("Threshold1", WINDOW_AUTOSIZE);
    createTrackbar("LowH", "Threshold1", &iLowH, 179);
    createTrackbar("HighH", "Threshold1", &iHighH, 179);
    createTrackbar("LowS", "Threshold1", &iLows, 225);
    createTrackbar("HighS", "Threshold1", &iHighs, 225);
    createTrackbar("LowV", "Threshold1", &iLowv, 225);
    createTrackbar("HighV", "Threshold1", &iHighv, 225);
    
    namedWindow("Threshold2", WINDOW_AUTOSIZE);
    createTrackbar("LowH2", "Threshold2", &iLowH2, 179);
    createTrackbar("HighH2", "Threshold2", &iHighH2, 179);
    createTrackbar("LowS2", "Threshold2", &iLows2, 225);
    createTrackbar("HighS2", "Threshold2", &iHighs2, 225);
    createTrackbar("LowV2", "Threshold2", &iLowv2, 225);
    createTrackbar("HighV2", "Threshold2", &iHighv2, 255);

    namedWindow("Threshold", WINDOW_AUTOSIZE);
    createTrackbar("Low", "Threshold", &Low, 255);
    createTrackbar("High", "Threshold", &High, 255);
}

class Image
{
    private:
       //-------------------------------结构体----------------------------------------
       // 模板数据结构：存储模板图像与标签
        struct TemplateData 
        {
            Mat image;       // 模板图像（灰度）
            string label;    // 模板标签
        };

        // 面特征结构：用于聚类和最优面选择
        struct FaceFeature 
        {
            int index;                  // 对应approxContours中的索引
            vector<Point2f> corners;    // 四个角点坐标
            Point2f center2D;           // 2D中心坐标（图像中）
            float area;                 // 图像中面积（像素）
            float matchVal;             // 模板匹配得分
            float score;                // 综合评分（用于选最优面）
            string parsedLabel;         // 解析后的标签
        };


        // DBSCAN聚类点结构
        struct DBSCANPoint 
        {
            int index;          // 对应FaceFeature的索引
            Point2f coord;      // 2D坐标
            bool visited;       // 是否被访问
            int clusterId;      // 所属簇ID（-1为噪声）
            string parsedLabel;
        };

        //-------------------------------相关参数的配置----------------------------------------
        string target;  //目标方块，从命令行获得

        double minArea = 225;  // 过滤小轮廓的面积阈值（像素）
        Size dstSize = Size(200, 200);  // 透视变换目标尺寸
        float mm2px = 3.0f;  // 毫米转像素比例

        //帧间稳定性控制
        const float SCORE_DIFF_THRESH = 0.05f; // 评分差异阈值：当前与历史面评分差小于此值，沿用历史面
        const float CENTER_DIST_THRESH = 20.0f; // 中心距离阈值：像素单位，小于此值视为同一物理目标

        //-------------------------------数据储存----------------------------------------
        vector<TemplateData> templates;  // 存储所有模板
        vector<FaceFeature> last_best_faces;  // 保存上一帧最优面列表，为下一帧继承做准备
        ros::NodeHandle nh;
        ros::Publisher faceinfo_pub;   // FaceInfo发布者（回调需发布消息）
        ros::Subscriber rgb_sub; //订阅相机图像

        //-------------------------------辅助函数----------------------------------------
        float distance2D(const Point2f& a, const Point2f& b);
        vector<vector<int>> simpleDBSCAN(vector<DBSCANPoint>& points, float eps, int minPts);
        void getChannel(const Mat &image);
        Mat warpSingleRegion(const Mat& origImg, const vector<Point2f>& srcPts, const Size& dstSize);
        string parseLabel(const string& label);
        int countCharHoles(const Mat& charBinary);

        //-------------------------------核心回调函数----------------------------------------
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        
    public:
        Image(const string& targetLabel, const string& nodeName);
        bool LoadTemplates(const string& dirPath);
};

// 计算两点间2D欧氏距离
float Image::distance2D(const Point2f& a, const Point2f& b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

// DBSCAN聚类：按标签和距离聚合同一物理方块的面
vector<vector<int>> Image::simpleDBSCAN(vector<DBSCANPoint>& points, float eps, int minPts) 
{
    int clusterId = 0;
    vector<vector<int>> clusters;  // 存储所有簇（每个簇是点的索引集合）

    for (size_t i = 0; i < points.size(); ++i) {
        if (points[i].visited) continue;  // 跳过已处理的点

        // 1. 寻找当前点的所有直接邻居（距离≤eps）
        vector<int> neighbors;
        for (size_t j = 0; j < points.size(); ++j) 
        {
            if (distance2D(points[i].coord, points[j].coord) <= eps && points[i].parsedLabel == points[j].parsedLabel) 
            {
                neighbors.push_back(j);  // 收集直接邻居（包括自身）
            }
        }

        // 2. 判断是否为核心点（邻居数≥minPts）
        if (neighbors.size() < minPts) 
        {
            // 非核心点：标记为噪声
            points[i].clusterId = -1;
        }
        else 
        {
            // 核心点：创建新簇，将自身和所有直接邻居纳入（未被处理的邻居）
            clusters.emplace_back();  // 新增一个空簇
            int currentCluster = clusterId;  // 当前簇ID

            // 遍历所有直接邻居，加入当前簇
            for (int neighborIdx : neighbors) 
            {
                // 仅处理未被其他簇收录的邻居
                if (!points[neighborIdx].visited) 
                {
                    points[neighborIdx].visited = true;  // 标记为已处理
                    points[neighborIdx].clusterId = currentCluster;  // 归属当前簇
                    clusters[currentCluster].push_back(points[neighborIdx].index);  // 加入簇
                }
            }
            clusterId++;  // 簇ID递增，准备下一个簇
        }
    }


    return clusters;
}

// 构造函数：初始化目标、ROS、模板
Image::Image(const string& targetLabel, const string& nodeName) 
    : target(targetLabel), nh(nodeName) //初始化成员变量
{
    // 初始化ROS发布者和订阅者
    faceinfo_pub = nh.advertise<image_msgs::FaceInfo>("cv_image", 10);
    rgb_sub = nh.subscribe("/camera/color/image_raw", 1, &Image::imageCallback, this); // 绑定this
    // 加载模板
    string templateDir = "/opt/ep_ws/src/rmus_solution/task1/temple";
    if (!LoadTemplates(templateDir)) 
    {
        throw runtime_error("Load templates is filed!");
    }
    ROS_INFO("Image init succeed ! Result is %s", target.c_str());
}

// 加载模板图像（从指定目录读取所有PNG文件）（类的成员函数的 “声明” 优先于 “定义” 被编译器处理）
bool Image::LoadTemplates(const string& dirPath) 
{
    if (!fs::exists(dirPath) || !fs::is_directory(dirPath)) {
        ROS_ERROR("The template directory does not exist : %s", dirPath.c_str());
        return false;
    }

    for (const auto& entry : fs::directory_iterator(dirPath)) {
        if (entry.is_regular_file() && entry.path().extension() == ".png") 
        {
            Mat temp = imread(entry.path().string(), IMREAD_GRAYSCALE);
            if (!temp.empty()) 
            {
                TemplateData td;
                td.image = temp;
                td.label = entry.path().stem().string();
                templates.push_back(td);
                ROS_INFO("Load the template: %s", td.label.c_str());
            }
        }
    }

    if (templates.empty()) 
    {
        ROS_WARN("The template directory is empty!");
        return false;
    } 
    else 
    {
        ROS_INFO("Loaded a total of %ld templates", templates.size());
        return true;
    }
}

// 打印图像尺寸和通道数
void Image::getChannel(const Mat &image) 
{
    if (image.empty()) {
        ROS_WARN("The image is empty");
        return;
    }
    int width = image.cols;
    int height = image.rows;
    int channels = image.channels();
    ROS_INFO("Image Size: Width=%d, Height=%d, Channels=%d", width, height, channels);
}

// 透视变换：矫正目标区域为矩形
Mat Image::warpSingleRegion(const Mat& origImg, const vector<Point2f>& srcPts, const Size& dstSize) 
{
    vector<Point2f> dstPts = {
    {0.0f, 0.0f}, 
    {(float)dstSize.width, 0.0f},
    {(float)dstSize.width, (float)dstSize.height},
    {0.0f, (float)dstSize.height}
};

    
    Mat persMat = getPerspectiveTransform(srcPts, dstPts);
    Mat warpedImg;
    warpPerspective(origImg, warpedImg, persMat, dstSize);
    return warpedImg;
}



// 标签解析函数
string Image::parseLabel(const string& label) 
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
    return label;
}

// 统计字符内部的孔洞数量
int Image::countCharHoles(const Mat& charBinary) 
{
    if (charBinary.empty() || charBinary.channels() != 1) {
        ROS_WARN("The character image is empty or has an incorrect number of channels");
        return -1;
    }

    int numComponents;
    Mat labels, stats, centroids;
    numComponents = connectedComponentsWithStats(
        charBinary, labels, stats, centroids, 4, CV_32S
    );

    const int MIN_HOLE_AREA = 40;  // 孔洞面积阈值（根据200x200图像调整）
    int validBgCount = 0;
    for (int i = 1; i < numComponents; ++i) {
        int area = stats.at<int>(i, CC_STAT_AREA);
        if (area > MIN_HOLE_AREA) {
            validBgCount++;
        }
    }

    return max(0, validBgCount - 1);  // 减去外部背景
}

// 图像回调函数（处理ROS图像消息）
void Image::imageCallback(const sensor_msgs::ImageConstPtr& msg) //(const string& imagePath)
{
        // // 1. 读取原始图像
        // Mat image_orign = imread(imagePath);
        // if(image_orign.empty())
        // {
        //     return;
        // }
        
        //将ROS图像消息转为OpenCV的Mat（替代原有的读文件）
        cv_bridge::CvImagePtr cv_ptr;
        try 
        {
            // 转换为BGR格式（OpenCV默认）
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } 
        catch (cv_bridge::Exception& e) 
        {
            ROS_ERROR("cv_bridge failed: %s", e.what());
            return;
        }

        Mat image_orign = cv_ptr->image;  // 得到OpenCV图像（无需再读文件）
        if (image_orign.empty()) 
        {
            ROS_WARN("image_orign is empty!");
            return;
        }

        // 2. HSV转换与V通道均衡化
        Mat hsv_image;
        cvtColor(image_orign, hsv_image, COLOR_BGR2HSV);
        vector<Mat> hsv_split;
        split(hsv_image, hsv_split);
        equalizeHist(hsv_split[2], hsv_split[2]);  // 增强亮度对比度
        merge(hsv_split, hsv_image);

        // 3. 红色区域阈值分割（HSV双范围）
        Mat mask1, mask2, redMask;
        inRange(hsv_image, Scalar(iLowH, iLows, iLowv), Scalar(iHighH, iHighs, iHighv), mask1);
        inRange(hsv_image, Scalar(iLowH2, iLows2, iLowv2), Scalar(iHighH2, iHighs2, iHighv2), mask2);
        redMask = mask1 | mask2;

        // 形态学操作去噪
        Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(redMask, redMask, MORPH_OPEN, element);  // 去噪点
        morphologyEx(redMask, redMask, MORPH_CLOSE, element); // 补孔洞

        // 4. 轮廓检测与筛选
        vector<vector<Point>> contours;
        findContours(redMask.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        
        vector<vector<Point>> validContours;
        for (size_t i = 0; i < contours.size(); ++i) 
        {
            double area = contourArea(contours[i]);
            if (area >= minArea) 
            {  // 过滤小轮廓
                validContours.push_back(contours[i]);
            }
        }
        if (validContours.empty()) 
        {
            return;
        }

        // 5. 多边形逼近提取四边形角点
        vector<vector<Point2f>> approxContours;
        vector<Point2f> approx;
        for (size_t i = 0; i < validContours.size(); ++i) 
        {
            double perimeter = arcLength(validContours[i], true); //轮廓周长
            approxPolyDP(validContours[i], approx, perimeter * 0.02, true);  // 2%周长精度(最佳精度)
            if (approx.size() == 4) 
            {  // 只保留四边形
                approxContours.push_back(approx);
            }
        }
        if (approxContours.empty()) 
        {
            return;
        }

        // 6. 提取面特征并进行模板匹配
        vector<FaceFeature> faceFeatures;
        for (size_t i = 0; i < approxContours.size(); ++i) 
        {
            // 计算2D中心（四角平均）
            Point2f center(0, 0);
            for (const auto& pt : approxContours[i])
            {
                center.x += pt.x;
                center.y += pt.y;
            }
            center.x /= 4;
            center.y /= 4;

            // 计算面积（像素）
            float area = contourArea(approxContours[i]);

            // 透视变换获取矫正图像
            Mat warpedImg = warpSingleRegion(redMask, approxContours[i], dstSize);
            // 二值化（将红mask转为纯黑白，消除灰度波动干扰）
            Mat warpedBinary;
            threshold(warpedImg, warpedBinary, 127, 255, THRESH_BINARY);

            // 模板匹配
            double bestMatchVal = 0.0;
            string recognizedLabel;
            for (const auto& temp : templates) 
            {
                //缩放模板
                Mat tempResized;
                resize(temp.image, tempResized, dstSize);
                //黑底白字 ---> 白底黑字
                Mat tempBinary;
                threshold(tempResized, tempBinary, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);

                //
                Mat matchResult;
                matchTemplate(warpedBinary, tempBinary, matchResult, TM_CCOEFF_NORMED);
                double maxVal;
                //找到最大值，其他都不关心
                minMaxLoc(matchResult, nullptr, &maxVal, nullptr, nullptr);

                if (maxVal > bestMatchVal) {
                    bestMatchVal = maxVal;
                    recognizedLabel = temp.label;
                }
            }

            // 标签解析与孔洞修正
            if (bestMatchVal > 0.6) 
            {  
                string parsedLabel = parseLabel(recognizedLabel);
                // 仅对高匹配度进行孔洞修正
                Mat charFromOrig = warpSingleRegion(image_orign, approxContours[i], dstSize);
                Mat charGray, charBinary;
                cvtColor(charFromOrig, charGray, COLOR_BGR2GRAY);
                threshold(charGray, charBinary, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);
                int holeNum = countCharHoles(charBinary);

                // 修正5/6/B（依赖孔洞数）
                if (parsedLabel == "5" || parsedLabel == "6" || parsedLabel == "B") 
                {
                    if (holeNum == 0) parsedLabel = "5";
                    else if (holeNum == 1) parsedLabel = "6";
                    else if (holeNum == 2) parsedLabel = "B";
                }

                // 存储面特征
                faceFeatures.push_back({
                    (int)i, approxContours[i], center, area, 
                    (float)bestMatchVal, 0.0f, parsedLabel
                });
            }

        }

        // 7. DBSCAN聚类（区分不同方块）
        // 聚类阈值：1.5倍方块边长（50mm）=75mm，转换为像素
        float eps = 50.0f * mm2px;  // 75mm * 4px/mm = 300px（需根据实际校准调整）
        int minPts = 1;  // 每个方块至少1个面

        vector<DBSCANPoint> dbscanPoints;
        for (size_t i = 0; i < faceFeatures.size(); ++i) 
        {
            dbscanPoints.push_back({
                (int)i, faceFeatures[i].center2D, false, -1, faceFeatures[i].parsedLabel
            });
        }

        vector<vector<int>> clusters = simpleDBSCAN(dbscanPoints, eps, minPts);
        // ROS_INFO("识别到%ld个方块",clusters.size());
        // for (size_t i = 0; i < clusters.size(); ++i) 
        // {
        //     ROS_INFO("簇 %ld 包含 %ld 个面", i, clusters[i].size());
        // }
        // ROS_INFO("共检测到 %ld 个四边形", approxContours.size());


       // 8. 为每个方块选择最优面（多指标加权 + 帧间稳定性优化）
        vector<FaceFeature> bestFaces;
        for (const auto& cluster : clusters) {
            if (cluster.empty()) continue;

            // 步骤1：过滤簇内无效面（仅保留匹配得分≥0.6的高置信度面，从源头减少干扰）
            vector<FaceFeature> validFacesInCluster;
            for (int idx : cluster) {
                if (faceFeatures[idx].matchVal >= 0.6) { 
                    validFacesInCluster.push_back(faceFeatures[idx]);
                }
            }
            if (validFacesInCluster.empty()) {
                ROS_WARN("No valid faces in the cluster (matching score < 0.6); skipping this cluster");
                continue;
            }

            // 步骤2：计算综合评分（匹配得分70% + 归一化面积30%，平衡准度与完整性）
            float maxAreaInCluster = 0.0f;
            for (const auto& face : validFacesInCluster) {
                maxAreaInCluster = max(maxAreaInCluster, face.area); // 求簇内最大面积，用于归一化
            }
            const float W_MATCH = 0.7f;  // 匹配得分权重（主导：优先保证识别准度）
            const float W_AREA = 0.3f;   // 面积权重（辅助：排除过小的不完整面）
            for (auto& face : validFacesInCluster) {
                float normalizedArea = face.area / maxAreaInCluster; // 面积归一化到[0,1]，消除量级差异
                face.score = W_MATCH * face.matchVal + W_AREA * normalizedArea; // 综合评分
            }

            // 步骤3：找到当前帧簇内最优面（综合评分最高）
            auto currentBestIt = max_element(validFacesInCluster.begin(), validFacesInCluster.end(),
                [](const FaceFeature& a, const FaceFeature& b) {
                    return a.score < b.score; 
                });
            FaceFeature currentBestFace = *currentBestIt;

            // 步骤4：帧间稳定性判断（差异小时沿用历史面，避免频繁切换）
            FaceFeature finalBestFace = currentBestFace;
            if (!last_best_faces.empty()) { // 若有历史帧数据，进行3重条件判断
                for (const auto& lastFace : last_best_faces) {
                    // 条件1：标签相同（确保是同一类目标）
                    if (lastFace.parsedLabel != currentBestFace.parsedLabel) continue;
                    // 条件2：中心距离近（确保是同一物理目标，避免跨目标混淆）
                    float centerDist = distance2D(lastFace.center2D, currentBestFace.center2D);
                    if (centerDist > CENTER_DIST_THRESH) continue;
                    // 条件3：评分差异小（当前帧无显著提升，沿用历史面更稳定）
                    if (fabs(currentBestFace.score - lastFace.score) < SCORE_DIFF_THRESH) {
                        finalBestFace = lastFace;
                        break;
                    }
                }
            }

            bestFaces.push_back(finalBestFace);
        }

        // 步骤5：更新历史帧数据（为下一帧的稳定性判断做准备）
        last_best_faces = bestFaces;

        //生成消息包
        image_msgs::FaceInfo msgs;
        for (const auto& bestFace : bestFaces) 
        {
            ROS_INFO("result: %s", bestFace.parsedLabel.c_str());
            // 只发布目标方块的消息（target是全局变量，从命令行传入）
            if (bestFace.parsedLabel == target) 
            {
                msgs.id = bestFace.parsedLabel;
                // 填充角点（注意：faceFeatures.corners是4个点，对应msgs.corner）
                for (size_t i = 0; i < bestFace.corners.size() && i < 4; ++i) 
                {
                    msgs.corner[i].x = bestFace.corners[i].x;
                    msgs.corner[i].y = bestFace.corners[i].y;
                }
                // 发布消息
                faceinfo_pub.publish(msgs);
                ROS_INFO("target is %s", target.c_str());
            }
        }

        // 9. 绘制最优面与角点ID
        for (const auto& bestFace : bestFaces) 
        {
            //终端输出识别的结果
            ROS_INFO("result:%s", bestFace.parsedLabel.c_str());
             

            // 标记角点ID（1-4）
            for (int j = 0; j < 4; ++j) 
            {
                Point2f corner = bestFace.corners[j];
                circle(image_orign, corner, 5, Scalar(0, 0, 255), -1);  // 红色角点

                // 角点ID文本（白字黑边）
                string idText = to_string(j+1);
                putText(image_orign, idText, corner + Point2f(5, -5),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);  // 白色文字
                
                //输出坐标
                ROS_INFO("num%d:(%.2f,%.2f)",(j+1),corner.x,corner.y);

                line(image_orign, bestFace.corners[j], bestFace.corners[(j+1)%4], Scalar(0, 255, 0), 1);  // 绿色边框
                
            }

            // 绘制标签
            Point textPos(bestFace.corners[1].x - 10, bestFace.corners[1].y - 10);
            putText(image_orign, bestFace.parsedLabel, textPos,
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 1);  // 蓝色标签
        }


        // //显示结果
        // imshow("bgr", image_orign);
        // imshow("hsv", redMask);
        // waitKey(5);

}


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"cv_image_node");
    ros::NodeHandle pnh("~");
    // 获取字符串参数（默认值为空）
    std::string number;
    pnh.param<std::string>("number", number, "2"); 
  
    Image image(number,"cv_image_node");
    ros::spin();
    
//    namedWindow("bgr");
//    namedWindow("hsv");
    //namedWindow("re");

//    while (1)
//    {
//         imageCallback("/home/qq/catkin_ws/src/task1/picture/target1.png");
//    }
   
    return 0;
}

