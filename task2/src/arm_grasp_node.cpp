#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

class ArmGraspController 
{
    private:
        ros::NodeHandle nh;
        ros::Subscriber nearby_sub;
        ros::Publisher arm_pos_pub;
        ros::Publisher arm_gripper_pub;
        bool is_grasping = false;

        // 与抓取的位置
        const double PRE_GRASP_X = 0.19;  
        const double PRE_GRASP_Y = -0.08; 
        const double PRE_GRASP_Z = 0.0;           

    public:
        ArmGraspController();  //析构函数
        void nearbyCallback(const std_msgs::Bool::ConstPtr& msg);
        void executeGraspSequence();
};

ArmGraspController::ArmGraspController() 
{
    // 订阅“已靠近”信号
    nearby_sub = nh.subscribe<std_msgs::Bool>("/robot/nearby_done", 10, &ArmGraspController::nearbyCallback, this);
    // 发布机械臂位姿和夹爪指令
    arm_pos_pub = nh.advertise<geometry_msgs::Pose>("/arm_position", 10);
    arm_gripper_pub = nh.advertise<geometry_msgs::Point>("/arm_gripper", 10);
}

void ArmGraspController::nearbyCallback(const std_msgs::Bool::ConstPtr& msg) 
{
    if(msg->data)
    {
        if(!is_grasping)
        {
            is_grasping = true;
            executeGraspSequence();
        }
    }
    else
    {
        ROS_INFO("Waiting for 'nearby' signal, starting grasp sequence...");
    }
}

void ArmGraspController::executeGraspSequence() 
{
    // 步骤1：移动到预抓取位姿
    geometry_msgs::Pose pre_grasp_pose;
    pre_grasp_pose.position.x = 0.19;
    pre_grasp_pose.position.y = -0.08;
    pre_grasp_pose.position.z = 0.0;
    pre_grasp_pose.orientation.x = 0.0;
    pre_grasp_pose.orientation.y = 0.0;
    pre_grasp_pose.orientation.z = 0.0;
    pre_grasp_pose.orientation.w = 0.0;
    arm_pos_pub.publish(pre_grasp_pose);
    ros::Duration(3.0).sleep();  // 等待机械臂运动到位
    // 步骤2：闭合夹爪
    geometry_msgs::Point gripper_close;
    gripper_close.x = 1.0;  // x=1表示关闭夹爪
    gripper_close.y = 0.0;
    gripper_close.z = 0.0;
    arm_gripper_pub.publish(gripper_close);
    ros::Duration(3.0).sleep();  // 等待夹爪动作完成
    // 步骤3：抬起机械臂（避免拖动方块）
    geometry_msgs::Pose lift_pose ;
    lift_pose.position.x = 0.19;
    lift_pose.position.y = 0.05;
    lift_pose.position.z = 0.0;
    pre_grasp_pose.orientation.x = 0.0;
    pre_grasp_pose.orientation.y = 0.0;
    pre_grasp_pose.orientation.z = 0.0;
    pre_grasp_pose.orientation.w = 0.0;
    arm_pos_pub.publish(lift_pose);
    ros::Duration(3.0).sleep();
    is_grasping = false;  // 重置状态，允许下次抓取
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "arm_grasp_node");
    ArmGraspController controller;
    ros::spin();
    return 0;
}
