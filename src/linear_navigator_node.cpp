#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud.h>

sensor_msgs/PointCloud lastPointCloud;
geometry_msg::Pose2D targetPose;
nav_msgs::Odometry lastOdom;

/*
geometry_msgs/Vector3 linear
  float64 x <-- 2D
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z <-- 2D
*/
void sendMotorSignal(){
	geometry_msgs::Twist& msg;
	ROS_INFO("target Left: %f, target Right: %f",t_WLeft,t_WRight);
}

void obstacleDisntacesCallback(const sensor_msgs/PointCloud& msg){
	if(msg.header.seq > lastPointCloud.header.seq){
		lastPointCloud = msg;
	}
}

/*
rosmsg show nav_msgs/Odometry 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance
*/
void currentPoseCallback(const nav_msgs::Odometry& msg){
	if(msg.header.seq > lastOdom.header.seq){
		lastOdom = msg;
	}
}

void targetPoseCallback(const geometry_msg::Pose2D& msg){
	targetPose = msg;
}

geometry_msgs::Twist calculateTwist(geometry_msgs::Pose& currentPose, geometry_msg::Pose2D& targetPose){
	
}

char checkCollisionCourse(geometry_msgs::Twist signal, sensor_msgs/PointCloud lastPointCloud, nav_msgs::Odometry lastOdom){
	
}

void moveTowardsPose(geometry_msgs::Pose& currentPose, geometry_msg::Pose2D& targetPose){
	geometry_msgs::Twist signal = calculateTwist();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_linear_navigator");

    ros::NodeHandle n;

    ros::Publisher motorSignal_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist",1);
    ros::Subscriber controller_sub = n.subscribe("/navigator/targetPose", 1, targetPoseCallback);
    ros::Subscriber controller_sub = n.subscribe("/odom", 1, currentPoseCallback);
	ros::Subscriber controller_sub = n.subscribe("/my_cloud", 1, obstacleDisntacesCallback);

    ros::Rate loop_rate(10);

    while(ros::ok()){
		moveTowardsPose(lastOdom.pose.pose, targetPose);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
