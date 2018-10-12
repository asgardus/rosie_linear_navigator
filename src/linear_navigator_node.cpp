#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <math.h>
#include <tf/transform_listener.h>

sensor_msgs::PointCloud lastPointCloud;
geometry_msgs::Pose2D targetPose;
nav_msgs::Odometry lastOdom;

char gotOdom = 0;
char gotTarget = 0;
char gotPointCloud = 0;

ros::Publisher motorSignal_pub;

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
float lastMotorLinearVelocitySignal = 0;
float lastMotorAngularVelocitySignal = 0;
void sendMotorSignal(geometry_msgs::Twist& msg){
	if(msg.linear.x != lastMotorLinearVelocitySignal || msg.angular.z != lastMotorAngularVelocitySignal){
		motorSignal_pub.publish(msg);
	}
	lastMotorLinearVelocitySignal = msg.linear.x;
	lastMotorAngularVelocitySignal = msg.angular.z;
}

void obstacleDisntacesCallback(const sensor_msgs::PointCloud& msg){
	if(msg.header.seq > lastPointCloud.header.seq){
		lastPointCloud = msg;
		gotPointCloud = 1;
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
	if(!gotOdom || msg.header.seq > lastOdom.header.seq){
		lastOdom = msg;
		gotOdom = 1;
	}
}

void targetPoseCallback(const geometry_msgs::Pose2D& msg){
	targetPose = msg;
	gotTarget = 1;
}

geometry_msgs::Twist calculateTwist(const nav_msgs::Odometry& currentOdom, const geometry_msgs::Pose2D& targetPose){

	geometry_msgs::Twist twistOut;
	
	tf::TransformListener listener;
	tf::StampedTransform transform;
	
	try{
		listener.lookupTransform("world", currentOdom.child_frame_id, ros::Time(0), transform);
	}catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	
	float worldX = transform.getOrigin().x();
	float worldY = transform.getOrigin().y();
	float deltaX = targetPose.x - worldX;
	float deltaY = targetPose.y - worldY;
	float deltaAnglePosition = atan2(deltaX, deltaY);
	float distance = sqrt(pow(deltaX,2) + pow(deltaY,2));
	
	if(distance > 0.05){
		float velByDistance = std::min(std::max((distance)*10,0.1f),0.3f);
		twistOut.linear.x = std::max(velByDistance * pow(cos(deltaAnglePosition),2),0.0);
		float angleDiff = (deltaAnglePosition);
		while(angleDiff < 0)
			angleDiff += 6.28319f;
		while(angleDiff > 6.28319f)
			angleDiff -= 6.28319f;
		if(angleDiff > 3.14159){
			angleDiff = (angleDiff-6.28319);
		}
		twistOut.angular.z = std::min(std::max(angleDiff,0.2f),1.0f);
	}else{
		tf::Quaternion deltaPoseQuaternion = transform.getRotation();
		tf::Matrix3x3 m(deltaPoseQuaternion);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		float angleDiff = (targetPose.theta-yaw);
		while(angleDiff < 0)
			angleDiff += 6.28319f;
		while(angleDiff > 6.28319f)
			angleDiff -= 6.28319f;
		if(angleDiff > 3.14159){
			angleDiff = (angleDiff-6.28319);
		}
		twistOut.linear.x = 0.0;
		twistOut.angular.z = std::min(std::max(angleDiff,0.2f),1.0f);
	}
	
	return twistOut;
}

char checkCollisionCourse(geometry_msgs::Twist signal, sensor_msgs::PointCloud lastPointCloud, nav_msgs::Odometry lastOdom){
	if(!gotPointCloud)
		return 1;
	float angleWidth = std::max(1.0 + abs(lastOdom.twist.twist.angular.z),1.5);
	float angleCenter = 3.1415+(lastOdom.twist.twist.angular.z*0.5);
	
	tf::TransformListener listener;
	tf::StampedTransform transform;
	
	for(int i = 0; i < 360; ++i){
		 geometry_msgs::Point32 pointInCloud = lastPointCloud.points[i];
		 
		try{
			listener.lookupTransform("base_link", lastPointCloud.header.frame_id, ros::Time(0), transform);
		}catch(tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
		
		tf::Vector3 point(pointInCloud.x,pointInCloud.y,pointInCloud.z);
		
		tf::Vector3 point_tf = transform * point;

		float angleFromBase = atan2(point_tf.x(), point_tf.y())-angleCenter;
		float distance = sqrt(pow(point_tf.x(),2)+pow(point_tf.y(),2));
		
		if(angleFromBase > angleWidth && angleFromBase < angleWidth){
				if(distance < 0.25){
					return 1;
				}
		}	 
	}
	return 0;
}

void moveTowardsPose(const nav_msgs::Odometry& currentOdom, const geometry_msgs::Pose2D& targetPose){
	geometry_msgs::Twist signal = calculateTwist(currentOdom, targetPose);
	if(checkCollisionCourse(signal, lastPointCloud, lastOdom)){
		signal.linear.x = 0;
		signal.angular.z = 0;
	}
	
	sendMotorSignal(signal);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_linear_navigator");

    ros::NodeHandle n;

    motorSignal_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist",1);
    ros::Subscriber targetPose_sub = n.subscribe("/navigator/targetPose", 1, targetPoseCallback);
    ros::Subscriber currentPose_sub = n.subscribe("/odom", 1, currentPoseCallback);
    ros::Subscriber obstacle_sub = n.subscribe("/my_cloud", 1, obstacleDisntacesCallback);

	sensor_msgs::PointCloud lastPointCloud_loc;
	geometry_msgs::Pose2D targetPose_loc;
	nav_msgs::Odometry lastOdom_loc;

	lastPointCloud = lastPointCloud_loc;
	targetPose = targetPose_loc;
	lastOdom = lastOdom_loc;

    ros::Rate loop_rate(10);

	lastOdom = nav_msgs::Odometry();

    while(ros::ok()){
		if(gotOdom && gotTarget){
			moveTowardsPose(lastOdom, targetPose);
		}
        ros::spinOnce();
        loop_rate.sleep();
    }
}
