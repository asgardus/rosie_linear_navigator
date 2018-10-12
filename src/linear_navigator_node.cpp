#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud.h>
#include <Math.h>
#include <tf/transform_broadcaster.h>

sensor_msgs/PointCloud lastPointCloud;
geometry_msg::Pose2D targetPose;
nav_msgs::Odometry lastOdom;
geometry_msgs::Twist lastSignal;

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
geometry_msgs::Twist& lastMotorSignal
void sendMotorSignal(geometry_msgs::Twist& msg){
	if(msg.linear.x != lastMotorSignal.linear.x || msg.angular.z != lastMotorSignal.angular.z){
		motorSignal_pub.publish(msg);
	}
	lastMotorSignal = msg;
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
	
	geometry_msgs::Twist twistOut;
	
	tf::TransformListener listener;
	tf::StampedTransform transform;
	
	try{
		listener.lookupTransform("world", currentPose.child_frame_id, ros::Time(0), transform);
	}catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	
	float worldX = transform.getOrigin().x();
	float worldY = transform.getOrigin().y();
	float deltaX = targetPose.x - worldX;
	float deltaX = targetPose.y - worldY;
	float deltaAnglePosition = atan2(deltaX, deltaY);
	float distance = sqrt(pow(deltaX,2),pow(deltaY,2));
	float distance = sqrt(pow(deltaX,2),pow(deltaY,2));
	
	if(distance > 0.05){
		float velByDistance = min(max((distance)*10,0.1),0.3);
		twistOut.linear.x = max(velByDistance * pow(cos(deltaAnglePosition),2),0);
		float angleDiff = (deltaAnglePosition)%6.28319;
		if(angleDiff > 3.14159){
			angleDiff = (angleDiff-6.28319);
		}
		twistOut.angular.z = min(max(angleDiff*K,0.2),1.0);
	}else{
		Quaternion deltaPoseQuaternion = transform.getRotation();
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		float angleDiff = (targetPose.theta-yaw)%6.28319;
		if(angleDiff > 3.14159){
			angleDiff = (angleDiff-6.28319);
		}
		twistOut.linear.x = 0.0;
		twistOut.angular.z = min(max(angleDiff*K,0.2),1.0);
	}
	
	return twistOut;
}

char checkCollisionCourse(geometry_msgs::Twist signal, sensor_msgs/PointCloud lastPointCloud, nav_msgs::Odometry lastOdom){
	float angleWidth = max(1.0 + abs(lastOdom.twist.twist.angular.z),1.5);
	float angleCenter = 3.1415+(lastOdom.twist.twist.angular.z*0.5);
	
	tf::TransformListener listener;
	tf::StampedTransform transform;
	
	for(int i = 0; i < 360; ++i){
		 geometry_msgs/Point point = lastPointCloud.points[i];
		 
		try{
			listener.lookupTransform("base_link", lastPointCloud.header.frame_id, ros::Time(0), transform);
		}catch(tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
		
		tf::Vector3 point(point.x,point.y,point.z);
		
		tf:Vector3 point_tf = transform * point;

		float angleFromTarget = atan2(point_tf.x, point_tf.y)-angleCenter;
		float distance = sqrt(pow(point.x,2)+pow(point.y,2));
		
		if(angleFromBase > angleWidth && angleFromBase < angleWidth){
				if(distance < 0.25){
					return 1;
				}
		}	 
	}
	return 0;
}

void moveTowardsPose(geometry_msgs::Pose& currentPose, geometry_msg::Pose2D& targetPose){
	geometry_msgs::Twist signal = calculateTwist();
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
