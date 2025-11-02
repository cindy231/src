// #include <ros/ros.h>
// #include <image_msgs/FaceInfo.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/imgproc.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <geometry_msgs/Twist.h>
// #include <dynamic_reconfigure/server.h>
// #include <task2/PIDConfig.h>  
// #include <std_msgs/Float64MultiArray.h>
// #include <std_msgs/Bool.h>


// using namespace cv;
// using namespace std;
// using namespace tf2;

// // 方块尺寸参数（单位：米）
// double L = 0.045;  
// vector<Point3f> modelPoints = 
// {
//     Point3f(-L/2, L/2, 0),   // 左上（自身坐标系）
//     Point3f(-L/2, -L/2, 0),  // 左下
//     Point3f(L/2, -L/2, 0),   // 右下
//     Point3f(L/2, L/2, 0)     // 右上
// };

// // 相机内参（需与实际相机配置一致）
// double fx = 617.305419921875;
// double fy = 617.305419921875;
// double cx = 424.0;             
// double cy = 240.0;             
// cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
//     fx, 0, cx,
//     0, fy, cy,
//     0, 0, 1
// );
// cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);  // 无畸变

// // 目标参数
// double target_distance = 0.26;    // x方向目标距离（米，底盘前向）
// double target_theta = 0.23;// 目标角度（弧度，0为正对方块）

// // PID控制器与发布器
// class PIDController;  
// PIDController* pid_v;  // x方向线速度PID（前进/后退）
// PIDController* pid_w;  // 角速度PID（旋转）
// ros::Publisher cmd_vel_pub;  
// ros::Publisher debug_pub;  
// ros::Publisher nearby_pub ;


// // 动态参数服务器
// dynamic_reconfigure::Server<task2::PIDConfig>* dr_server;
// dynamic_reconfigure::Server<task2::PIDConfig>::CallbackType dr_callback;


// // 复合坐标变换计算
// geometry_msgs::Transform composeTransform(const geometry_msgs::Transform& A, const geometry_msgs::Transform& B) 
// {
//     geometry_msgs::Transform C;
//     // 提取四元素
//     Quaternion qA(A.rotation.x, A.rotation.y, A.rotation.z, A.rotation.w);
//     Quaternion qB(B.rotation.x, B.rotation.y, B.rotation.z, B.rotation.w);
//     // 旋转复合：（先应用B的旋转，再应用A的旋转）
//     Quaternion qC = qA * qB; 
//     // 归一化四元数 
//     qC.normalize();  
//     C.rotation.x = qC.x();
//     C.rotation.y = qC.y();
//     C.rotation.z = qC.z();
//     C.rotation.w = qC.w();

//     // 平移复合：A的平移 + A旋转作用后的B平移
//     Vector3 tB(B.translation.x, B.translation.y, B.translation.z);
//     Vector3 tC = quatRotate(qA, tB) + Vector3(A.translation.x, A.translation.y, A.translation.z);
//     C.translation.x = tC.x();  
//     C.translation.y = tC.y();
//     C.translation.z = tC.z();
//     return C;
// }


// // PID控制器类（简化，仅保留必要功能）
// class PIDController {
// private:
//     double kp, ki, kd;  
//     double error_sum;   // 误差积分
//     double derror;      // 误差微分
//     double prev_error;  // 上一时刻误差
//     ros::Time last_time;  
//     bool is_first_run;  

// public:
//     PIDController(double Kp, double Ki, double Kd) : kp(Kp), ki(Ki), kd(Kd) 
//     {
//         reset();  
//     }

//     void reset() 
//     {
//         error_sum = 0;
//         derror = 0;
//         prev_error = 0;
//         last_time = ros::Time::now();
//         is_first_run = true;
//     }

//     double clamp(double value, double min_val, double max_val) 
//     {
//         return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
//     }

//     double update(double current_error, double& dt_out, double limit_min, double limit_max) 
//     {
//         ros::Time now = ros::Time::now();  //获取当前时间戳
//         double dt = (now - last_time).toSec();  // 计算与上一次控制的时间间隔（单位：秒）
//         last_time = now;
//         dt_out = dt;  

//         if (is_first_run || dt <= 0.001 || dt > 0.2) {
//             is_first_run = false;
//             prev_error = current_error;
//             return 0.0;  
//         }

//         // 比例项
//         double proportional = kp * current_error;

//         // 积分项（抗饱和）
//         error_sum += current_error * dt;
//         error_sum = (ki != 0) ? clamp(error_sum, -1.0 / ki, 1.0 / ki) : 0;
//         double integral_term = ki * error_sum;

//         // 微分项
//         derror = (current_error - prev_error) / dt ;
//         prev_error = current_error;
//         double derivative_term = kd * derror;

//         // 总输出（限幅）
//         double output = proportional + integral_term + derivative_term;
//         output = clamp(output, limit_min, limit_max);
//         return output;
//     }

//     void setPIDParams(double Kp, double Ki, double Kd) 
//     {
//         kp = Kp;
//         ki = Ki;
//         kd = Kd;
//     }

// };

// string grasp_state = "uncompleted";

// // 动态参数回调（仅保留x和w方向参数，删除y方向）
// void drCallback(task2::PIDConfig& config, uint32_t level) 
// {
//     ROS_INFO("参数更新: kp_x=%.3f, ki_x=%.3f, kd_x=%.3f | kp_w=%.3f, ki_w=%.3f, kd_w=%.3f | target_x=%.2f",
//              config.kp_x, config.ki_x, config.kd_x,
//              config.kp_w, config.ki_w, config.kd_w,
//              config.target_distance);

//     // 更新PID参数
//     pid_v->setPIDParams(config.kp_x, config.ki_x, config.kd_x);
//     pid_w->setPIDParams(config.kp_w, config.ki_w, config.kd_w);

//     // 更新目标参数
//     target_distance = config.target_distance;
//     target_theta = config.target_theta;
// }


// // 图像角点回调（核心控制逻辑）
// void faceInfoCallback(const image_msgs::FaceInfo::ConstPtr& msg) {
//     // 1. 解析图像角点
//     vector<Point2f> imagePoints(4);
//     for (int i = 0; i < 4; ++i) {
//         imagePoints[i].x = msg->corner[i].x;
//         imagePoints[i].y = msg->corner[i].y;
//     }

//     // 2. 角点排序（确保与modelPoints顺序一致：左上→左下→右下→右上）
//     // 按x坐标分组（左半部分0-1，右半部分2-3）
//     sort(imagePoints.begin(), imagePoints.end(), [](const Point2f& a, const Point2f& b) {
//         return a.x < b.x;
//     });
//     // 左半部分：y值大的为左上（0），y值小的为左下（1）
//     if (imagePoints[0].y < imagePoints[1].y) swap(imagePoints[0], imagePoints[1]);
//     // 右半部分：y值大的为右上（3），y值小的为右下（2）
//     if (imagePoints[2].y > imagePoints[3].y) swap(imagePoints[2], imagePoints[3]);

//     // 3. PnP解算位姿（相机光学系下）
//     Mat rvec, tvec;
//     bool success = solvePnP(
//         modelPoints, imagePoints, cameraMatrix, distCoeffs,
//         rvec, tvec, false, SOLVEPNP_ITERATIVE
//     );
//     if (!success) {
//         ROS_WARN_THROTTLE(1, "PnP解算失败，无法获取方块位姿");
//         return;
//     }

//     // 4. 转换为旋转矩阵和位姿矩阵
//     Mat R;  
//     Rodrigues(rvec, R);  
//     Mat poseMatrix = (Mat_<double>(4,4) <<
//         R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), tvec.at<double>(0),
//         R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), tvec.at<double>(1),
//         R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), tvec.at<double>(2),
//         0, 0, 0, 1
//     );

//     // 5. 相机光学系→底盘系（base_link）的变换（关键修正）
//     geometry_msgs::Transform optical_to_base;
//     // 平移：相机在底盘上的安装位置（x前向，y左向，z向上）
//     optical_to_base.translation.x = 0.16;   // 相机距底盘中心的前向距离
//     optical_to_base.translation.y = 0.0175;    // 居中安装，y向偏移为0
//     optical_to_base.translation.z = 0.025; // 相机高度
//     // 旋转：修正相机与底盘的坐标方向差异（核心！）
//     // 相机光学系默认：x右、y下、z前；底盘系：x前、y左、z上 → 绕z轴旋转90度对齐
//     Quaternion q_optical_base;
//     q_optical_base.setRPY(0, 0, 0);  // 绕z轴旋转90度（根据实际安装微调）
//     // 【修正1：Quaternion不支持列表初始化，改为逐个成员赋值】
//     optical_to_base.rotation.x = q_optical_base.x();
//     optical_to_base.rotation.y = q_optical_base.y();
//     optical_to_base.rotation.z = q_optical_base.z();
//     optical_to_base.rotation.w = q_optical_base.w();

//     // 6. 计算方块在底盘系下的位姿
//     geometry_msgs::Transform block_in_optical;
//     block_in_optical.translation.x = poseMatrix.at<double>(0, 3);
//     block_in_optical.translation.y = poseMatrix.at<double>(1, 3);
//     block_in_optical.translation.z = poseMatrix.at<double>(2, 3);
//     // 旋转矩阵→四元数
//     Matrix3x3 rotMat_optical(
//         poseMatrix.at<double>(0,0), poseMatrix.at<double>(0,1), poseMatrix.at<double>(0,2),
//         poseMatrix.at<double>(1,0), poseMatrix.at<double>(1,1), poseMatrix.at<double>(1,2),
//         poseMatrix.at<double>(2,0), poseMatrix.at<double>(2,1), poseMatrix.at<double>(2,2)
//     );
//     Quaternion quat;
//     rotMat_optical.getRotation(quat);
//     // 【修正2：删除错误的大括号+分号组合，改为直接成员赋值】
//     block_in_optical.rotation.x = quat.x();
//     block_in_optical.rotation.y = quat.y();
//     block_in_optical.rotation.z = quat.z();
//     block_in_optical.rotation.w = quat.w();

//     // 复合变换：方块在底盘系的位姿
//     geometry_msgs::Transform block_in_base = composeTransform(optical_to_base, block_in_optical);

//     // 7. 发布TF（base_link→block_frame）
//     static tf2_ros::TransformBroadcaster br;
//     geometry_msgs::TransformStamped transformStamped;
//     transformStamped.header.stamp = ros::Time::now();
//     transformStamped.header.frame_id = "base_link";
//     transformStamped.child_frame_id = "block_frame";
//     transformStamped.transform = block_in_base;
//     br.sendTransform(transformStamped);

//     // 8. 计算控制误差 方块距离底盘中心的距离
//     double distance = sqrt(
//         pow(block_in_base.translation.x,2) +
//         pow(block_in_base.translation.y,2) +
//         pow(block_in_base.translation.z,2)
//     );  
//    //提取yaw角（偏航角）
//    double yaw_rad = atan2(block_in_base.translation.y, block_in_base.translation.x);
    
//     // 误差计算（目标-当前）
//     double e_distance = distance - target_distance;  // x方向误差（需靠近/远离目标）
//     double e_theta = yaw_rad - target_theta;
//     e_theta = atan2(sin(e_theta), cos(e_theta));  // 归一化到[-π, π]

//     // 9. PID计算
//     double dt_v, dt_w;
//     double v_x = pid_v->update(e_distance, dt_v, -0.25, 0.25);    // x方向线速度
//     double w_z = pid_w->update(e_theta, dt_w, -0.5, 0.5);  // 角速度

   
//     // 11. 发布速度指令（差速轮机器人y向速度固定为0）
//     geometry_msgs::Twist cmd_vel;
//     //先旋转在靠近
//     if(fabs(e_theta) < 0.03 )
//     {
//         pid_w->reset();
//         cmd_vel.angular.z = 0;
//         cmd_vel.linear.x = v_x;
//     }
//     else
//     {
//         cmd_vel.angular.z = w_z;
//         cmd_vel.linear.x = 0.0;  // 旋转时不前进，确保对准
//     }
    
//     if (fabs(e_distance) < 0.03 ) 
//     {
//         pid_v->reset();
       
//     }
//     cmd_vel_pub.publish(cmd_vel);

//     // 发布已经靠近的信号
//     std_msgs::Bool nearby_msg;

//     // 判断是否处于"已停止"状态（线速度和角速度都为0）
//     bool is_stopped = (fabs(cmd_vel.linear.x) == 0.0) && (fabs(cmd_vel.angular.z) == 0.0);
//     if (is_stopped) 
//     {
//         // 情况1：机器人已停止，且未发送过"已靠近"信号
//         if (grasp_state == "uncompleted") 
//         {
//             nearby_msg.data = true;  // 首次发送true
//             nearby_pub.publish(nearby_msg);
//             grasp_state = "completed";  // 标记为已发送
//             ROS_INFO("已发送一次'已靠近'信号");
//         }
//         // 情况2：机器人已停止，且已发送过"已靠近"信号
//         else 
//         {
//             nearby_msg.data = false;  
//             nearby_pub.publish(nearby_msg);
//         }
//     } 
//     // 情况3：机器人正在移动（未靠近），发布false并重置状态
//     else 
//     {
//         nearby_msg.data = false;
//         nearby_pub.publish(nearby_msg);
//         grasp_state = "uncompleted";  // 重置标记，允许下次靠近时触发
//     }

    

//     // // 12. 发布调试数据
//     // std_msgs::Float64MultiArray debug_msg;
//     // debug_msg.data = {
//     //     target_distance, target_theta,  // 目标值
//     //     e_distance, e_theta,            // 误差
//     //     v_x, w_z,                // PID输出
//     //     distance, yaw_rad,                  // 当前测量值
//     //     dt_v                     // 时间步
//     // };
//     // debug_pub.publish(debug_msg);

//     // 打印日志
//     ROS_INFO_THROTTLE(0.5, "e_x=%.3fm, e_angle=%.3f | v_z=%.3f,w_z=%.3f | distance = %.3fm, yaw_deg = %.3f",
//              e_distance, e_theta, cmd_vel.linear.x, cmd_vel.angular.z, distance,yaw_rad);
    
// }


// int main(int argc, char *argv[]) 
// {
//     ros::init(argc, argv, "block_tf_pid_controller");
//     ros::NodeHandle nh;

//     // 初始化发布器
//     cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
//     debug_pub = nh.advertise<std_msgs::Float64MultiArray>("/tf_pid_node/debug", 10);
//     nearby_pub = nh.advertise<std_msgs::Bool>("/robot/nearby_done", 10);


//     // 初始化PID控制器（差速轮专用参数）
//     double kp_x = 0.0, ki_x = 0.0, kd_x = 0.0;  // x方向（前进后退）
//     double kp_w = 0.0, ki_w = 0.0, kd_w = 0.0;  // 角速度（旋转）
//     pid_v = new PIDController(kp_x, ki_x, kd_x);
//     pid_w = new PIDController(kp_w, ki_w, kd_w);
//     // 初始化动态参数服务器
//     dr_server = new dynamic_reconfigure::Server<task2::PIDConfig>();
//     dr_callback = boost::bind(&drCallback, _1, _2);
//     dr_server->setCallback(dr_callback);

//     // 订阅图像角点话题（需与实际发布话题一致）
//     ros::Subscriber sub = nh.subscribe<image_msgs::FaceInfo>("cv_image", 10, faceInfoCallback);

//     ROS_INFO("差速轮适配版节点启动成功！可通过rqt_reconfigure实时调参");

//     ros::spin();

//     // 释放资源
//     delete pid_v;
//     delete pid_w;
//     delete dr_server;
//     return 0;
// }


#include <ros/ros.h>
#include <image_msgs/FaceInfo.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <task2/PIDConfig.h>  
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

using namespace cv;
using namespace std;
using namespace tf2;

class PIDController;

class BlockController
{
    private:
        //方块尺寸参数（单位：米）
        double L;
        vector<Point3f> modelPoints;

        //相机的内置参数
        double fx, fy, cx, cy;
        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;

        // 目标参数
        double target_distance;    // x方向目标距离（米，底盘前向）
        double target_theta;       // 目标角度（弧度，0为正对方块）

        PIDController* pid_v;      // 线速度PID
        PIDController* pid_w;      // 角速度PID（旋转）
        ros::Publisher cmd_vel_pub;// 速度指令发布器
        //ros::Publisher debug_pub;  // 调试数据发布器
        ros::Publisher nearby_pub; // 靠近完成信号发布器
        ros::Subscriber face_info_sub;// 图像角点订阅器
        dynamic_reconfigure::Server<task2::PIDConfig>* dr_server; // 动态参数服务器
        tf2_ros::TransformBroadcaster tf_broadcaster; // TF广播器

        // 状态变量
        string grasp_state;        // 抓取状态标记（uncompleted/completed）
        geometry_msgs::Transform optical_to_base; // 相机→底盘变换矩阵
    
    public:
        // 构造函数：初始化参数、对象和通信接口
        BlockController(ros::NodeHandle& nh);
        // 析构函数：释放动态分配资源
        ~BlockController();
        // 图像角点回调：核心控制逻辑（位姿解算、PID计算、指令发布）
        void faceInfoCallback(const image_msgs::FaceInfo::ConstPtr& msg);
        // 动态参数回调：更新PID参数和目标参数
        void drCallback(task2::PIDConfig& config, uint32_t level);
        // 复合坐标变换计算：A变换 * B变换
        geometry_msgs::Transform composeTransform(const geometry_msgs::Transform& A, const geometry_msgs::Transform& B);
        // 角点排序: 确保与modelPoints顺序一致
        void sortImagePoints(vector<Point2f>& imagePoints);

};

// PID控制器类
class PIDController 
{
private:
    double kp, ki, kd;         // PID参数
    double error_sum;          // 误差积分
    double derror;             // 误差微分
    double prev_error;         // 上一时刻误差
    ros::Time last_time;       // 上一次控制时间戳
    bool is_first_run;         // 首次运行标记

public:
    // 构造函数：初始化PID参数和状态
    PIDController(double Kp, double Ki, double Kd);
    // 重置PID状态（误差积分、微分、时间戳等）
    void reset();
    // 限幅函数：将值限制在[min_val, max_val]范围内
    double clamp(double value, double min_val, double max_val);
    // PID更新：输入当前误差，输出控制量，返回时间间隔
    double update(double current_error, double& dt_out, double limit_min, double limit_max);
    // 设置PID参数：动态更新Kp、Ki、Kd
    void setPIDParams(double Kp, double Ki, double Kd);
};

// ------------------------------ BlockController类实现 ------------------------------
BlockController::BlockController(ros::NodeHandle& nh)
{
    // 1. 初始化方块尺寸与模型点
    L = 0.045;
    modelPoints = 
    {
        Point3f(-L/2, L/2, 0),   // 左上（自身坐标系）
        Point3f(-L/2, -L/2, 0),  // 左下
        Point3f(L/2, -L/2, 0),   // 右下
        Point3f(L/2, L/2, 0)     // 右上
    };

    // 2. 初始化相机内参
    fx = 617.305419921875;
    fy = 617.305419921875;
    cx = 424.0;             
    cy = 240.0;             
    cameraMatrix = (cv::Mat_<double>(3, 3) <<
        fx, 0, cx,
        0, fy, cy,
        0, 0, 1
    );
    distCoeffs = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);  // 无畸变

    // 3. 初始化目标参数
    target_distance = 0.26;
    target_theta = 0.23;

    // 4. 初始化通信接口
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    //debug_pub = nh.advertise<std_msgs::Float64MultiArray>("/tf_pid_node/debug", 10);
    nearby_pub = nh.advertise<std_msgs::Bool>("/robot/nearby_done", 10);
    face_info_sub = nh.subscribe<image_msgs::FaceInfo>("cv_image", 10, &BlockController::faceInfoCallback, this);

    // 5. 初始化PID控制器
    pid_v = new PIDController(0.0, 0.0, 0.0);  // 初始参数为0，通过动态参数更新
    pid_w = new PIDController(0.0, 0.0, 0.0);

    // 6. 初始化动态参数服务器
    dr_server = new dynamic_reconfigure::Server<task2::PIDConfig>();
    dynamic_reconfigure::Server<task2::PIDConfig>::CallbackType dr_cb = 
        boost::bind(&BlockController::drCallback, this, _1, _2);
    dr_server->setCallback(dr_cb);

    // 7. 初始化相机→底盘变换矩阵（关键修正）
    // 平移：相机在底盘上的安装位置（x前向，y左向，z向上）
    optical_to_base.translation.x = 0.16;   // 相机距底盘中心的前向距离
    optical_to_base.translation.y = 0.0175; // 居中安装，y向偏移
    optical_to_base.translation.z = 0.025;  // 相机高度
    // 旋转：相机光学系（x右、y下、z前）→底盘系（x前、y左、z上）
    Quaternion q_optical_base;
    q_optical_base.setRPY(0, 0, 0);  // 可根据实际安装微调旋转角度
    optical_to_base.rotation.x = q_optical_base.x();
    optical_to_base.rotation.y = q_optical_base.y();
    optical_to_base.rotation.z = q_optical_base.z();
    optical_to_base.rotation.w = q_optical_base.w();

    // 8. 初始化状态变量
    grasp_state = "uncompleted";
}

void BlockController::sortImagePoints(vector<Point2f>& imagePoints)
{
    // 替换sort为for循环：按x坐标从小到大排序（左半部分0-1，右半部分2-3）
    for (int i = 0; i < 3; ++i)
    {
        for (int j = i + 1; j < 4; ++j)
        {
            if (imagePoints[j].x < imagePoints[i].x)
            {
                swap(imagePoints[i], imagePoints[j]);
            }
        }
    }

    // 左半部分（0和1）：y值大的为左上（0），y值小的为左下（1）
    if (imagePoints[0].y < imagePoints[1].y)
    {
        swap(imagePoints[0], imagePoints[1]);
    }

    // 右半部分（2和3）：y值大的为右上（3），y值小的为右下（2）
    if (imagePoints[2].y > imagePoints[3].y)
    {
        swap(imagePoints[2], imagePoints[3]);
    }
}

BlockController::~BlockController()
{
    // 释放动态分配的资源，避免内存泄漏
    delete pid_v;
    delete pid_w;
    delete dr_server;
}

geometry_msgs::Transform BlockController::composeTransform(const geometry_msgs::Transform& A, const geometry_msgs::Transform& B) 
{
    geometry_msgs::Transform C;
    // 提取四元素
    Quaternion qA(A.rotation.x, A.rotation.y, A.rotation.z, A.rotation.w);
    Quaternion qB(B.rotation.x, B.rotation.y, B.rotation.z, B.rotation.w);
    // 旋转复合：（先应用B的旋转，再应用A的旋转）
    Quaternion qC = qA * qB; 
    // 归一化四元数 
    qC.normalize();  
    C.rotation.x = qC.x();
    C.rotation.y = qC.y();
    C.rotation.z = qC.z();
    C.rotation.w = qC.w();

    // 平移复合：A的平移 + A旋转作用后的B平移
    Vector3 tB(B.translation.x, B.translation.y, B.translation.z);
    Vector3 tC = quatRotate(qA, tB) + Vector3(A.translation.x, A.translation.y, A.translation.z);
    C.translation.x = tC.x();  
    C.translation.y = tC.y();
    C.translation.z = tC.z();
    return C;
}

void BlockController::faceInfoCallback(const image_msgs::FaceInfo::ConstPtr& msg)
{
    // 1. 解析图像角点
    vector<Point2f> imagePoints(4);
    for (int i = 0; i < 4; ++i)
    {
        imagePoints[i].x = msg->corner[i].x;
        imagePoints[i].y = msg->corner[i].y;
    }

    // 2. 角点排序（用for循环实现，替换原sort逻辑）
    sortImagePoints(imagePoints);

    // 3. PnP解算位姿（相机光学系下）
    Mat rvec, tvec;
    bool success = solvePnP(
        modelPoints, imagePoints, cameraMatrix, distCoeffs,
        rvec, tvec, false, SOLVEPNP_ITERATIVE
    );
    if (!success)
    {
        ROS_WARN_THROTTLE(1, "PnP解算失败，无法获取方块位姿");
        return;
    }

    // 4. 转换为旋转矩阵和位姿矩阵
    Mat R;  
    Rodrigues(rvec, R);  
    Mat poseMatrix = (Mat_<double>(4,4) <<
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), tvec.at<double>(0),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), tvec.at<double>(1),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), tvec.at<double>(2),
        0, 0, 0, 1
    );

    // 5. 构建方块在相机光学系下的变换
    geometry_msgs::Transform block_in_optical;
    block_in_optical.translation.x = poseMatrix.at<double>(0, 3);
    block_in_optical.translation.y = poseMatrix.at<double>(1, 3);
    block_in_optical.translation.z = poseMatrix.at<double>(2, 3);
    // 旋转矩阵→四元数
    Matrix3x3 rotMat_optical(
        poseMatrix.at<double>(0,0), poseMatrix.at<double>(0,1), poseMatrix.at<double>(0,2),
        poseMatrix.at<double>(1,0), poseMatrix.at<double>(1,1), poseMatrix.at<double>(1,2),
        poseMatrix.at<double>(2,0), poseMatrix.at<double>(2,1), poseMatrix.at<double>(2,2)
    );
    Quaternion quat;
    rotMat_optical.getRotation(quat);
    block_in_optical.rotation.x = quat.x();
    block_in_optical.rotation.y = quat.y();
    block_in_optical.rotation.z = quat.z();
    block_in_optical.rotation.w = quat.w();

    // 6. 计算方块在底盘系下的位姿（相机→底盘变换复合）
    geometry_msgs::Transform block_in_base = composeTransform(optical_to_base, block_in_optical);

    // 7. 发布TF（base_link→block_frame）
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "block_frame";
    transformStamped.transform = block_in_base;
    tf_broadcaster.sendTransform(transformStamped);

    // 8. 计算控制误差
    double distance = sqrt(
        pow(block_in_base.translation.x,2) +
        pow(block_in_base.translation.y,2) +
        pow(block_in_base.translation.z,2)
    );  
    double yaw_rad = atan2(block_in_base.translation.y, block_in_base.translation.x);  // 提取yaw角（偏航角）
    
    // 误差计算（目标-当前），并归一化角度到[-π, π]
    double e_distance = distance - target_distance;
    double e_theta = yaw_rad - target_theta;
    e_theta = atan2(sin(e_theta), cos(e_theta));

    // 9. PID计算控制量
    double dt_v, dt_w;
    double v_x = pid_v->update(e_distance, dt_v, -0.25, 0.25);    // x方向线速度（限幅±0.25）
    double w_z = pid_w->update(e_theta, dt_w, -0.5, 0.5);        // 角速度（限幅±0.5）

    // 10. 速度指令逻辑：先旋转对准，再前进靠近
    geometry_msgs::Twist cmd_vel;
    if (fabs(e_theta) < 0.03)  // 角度误差小于0.03弧度（约1.7°），视为对准
    {
        pid_w->reset();        // 重置角速度PID，避免积分累积
        cmd_vel.angular.z = 0;
        cmd_vel.linear.x = v_x; // 仅输出线速度，前进/后退
    }
    else  // 角度未对准，仅旋转
    {
        cmd_vel.angular.z = w_z;
        cmd_vel.linear.x = 0.0;
    }
    // 距离误差小于0.03米，重置线速度PID
    if (fabs(e_distance) < 0.03)
    {
        pid_v->reset();
    }
    cmd_vel_pub.publish(cmd_vel);

    // 11. 发布靠近完成信号：仅在首次停止时发送true
    std_msgs::Bool nearby_msg;
    bool is_stopped = (fabs(cmd_vel.linear.x) == 0.0) && (fabs(cmd_vel.angular.z) == 0.0);
    if (is_stopped)
    {
        if (grasp_state == "uncompleted")  // 首次停止，发送完成信号
        {
            nearby_msg.data = true;
            nearby_pub.publish(nearby_msg);
            grasp_state = "completed";
            ROS_INFO("已发送一次'已靠近'信号");
        }
        else  // 已发送过，后续停止发送false
        {
            nearby_msg.data = false;
            nearby_pub.publish(nearby_msg);
        }
    }
    else  // 机器人移动中，发送false并重置状态
    {
        nearby_msg.data = false;
        nearby_pub.publish(nearby_msg);
        grasp_state = "uncompleted";
    }

    // 12. 打印运行日志（0.5秒一次，避免刷屏）
    ROS_INFO_THROTTLE(0.5, "e_x=%.3fm, e_angle=%.3f | v_x=%.3f,w_z=%.3f | distance=%.3fm, yaw=%.3f",
        e_distance, e_theta, cmd_vel.linear.x, cmd_vel.angular.z, distance, yaw_rad);
}

// ------------------------------ BlockController类实现 ------------------------------
// 构造函数实现
PIDController::PIDController(double Kp, double Ki, double Kd)
    : kp(Kp), ki(Ki), kd(Kd)
{
    reset();  // 初始化PID内部状态
}

void BlockController::drCallback(task2::PIDConfig& config, uint32_t level)
{
    ROS_INFO("参数更新: kp_x=%.3f, ki_x=%.3f, kd_x=%.3f | kp_w=%.3f, ki_w=%.3f, kd_w=%.3f | target_x=%.2f",
        config.kp_x, config.ki_x, config.kd_x,
        config.kp_w, config.ki_w, config.kd_w,
        config.target_distance);

    // 更新PID参数
    pid_v->setPIDParams(config.kp_x, config.ki_x, config.kd_x);
    pid_w->setPIDParams(config.kp_w, config.ki_w, config.kd_w);

    // 更新目标距离参数
    target_distance = config.target_distance;
    target_theta = config.target_theta;
}

void PIDController::reset() 
{
    error_sum = 0;
    derror = 0;
    prev_error = 0;
    last_time = ros::Time::now();
    is_first_run = true;
}

double PIDController::clamp(double value, double min_val, double max_val) 
{
    return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
}

double PIDController::update(double current_error, double& dt_out, double limit_min, double limit_max) 
{
    ros::Time now = ros::Time::now();  //获取当前时间戳
    double dt = (now - last_time).toSec();  // 计算与上一次控制的时间间隔（单位：秒）
    last_time = now;
    dt_out = dt;  
    if (is_first_run || dt <= 0.001 || dt > 0.2) {
        is_first_run = false;
        prev_error = current_error;
        return 0.0;  
    }
    // 比例项
    double proportional = kp * current_error;
    // 积分项（抗饱和）
    error_sum += current_error * dt;
    error_sum = (ki != 0) ? clamp(error_sum, -1.0 / ki, 1.0 / ki) : 0;
    double integral_term = ki * error_sum;
    // 微分项
    derror = (current_error - prev_error) / dt ;
    prev_error = current_error;
    double derivative_term = kd * derror;
    // 总输出（限幅）
    double output = proportional + integral_term + derivative_term;
    output = clamp(output, limit_min, limit_max);
    return output;
}
void PIDController::setPIDParams(double Kp, double Ki, double Kd) 
{
    kp = Kp;
    ki = Ki;
    kd = Kd;
}

int main(int argc, char *argv[])
{
    // 初始化ROS节点
    ros::init(argc, argv, "block_tf_pid_controller");
    ros::NodeHandle nh;
    
    // 创建控制器实例
    BlockController controller(nh);
    
    // 进入ROS事件循环
    ros::spin();
    
    return 0;
}
