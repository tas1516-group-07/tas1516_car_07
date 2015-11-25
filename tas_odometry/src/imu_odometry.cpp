//Roman, TAS-GRP7, V1
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/LinkStates.h>



ros::Publisher odom_pub;
tf::TransformBroadcaster* odom_broadcaster;
nav_msgs::Odometry lastOdom; //Speichern der letzten Odometry


void saveOdometry (nav_msgs::Odometry* odomPntr, ros::Time stamp, double x, double y, geometry_msgs::Quaternion orientation){
	odomPntr->header.stamp = stamp;
	odomPntr->pose.pose.position.x=x;
	odomPntr->pose.pose.position.y=y;
	odomPntr->pose.pose.orientation=orientation;
}

void imuCallBack(const sensor_msgs::Imu::ConstPtr& imu) //Erhaelt Msg vom Typ sensor_msgs/Imu von Topic /imu
{
	ROS_INFO("received IMU message");
	double newX, newY, newLinVeloX, newLinVeloY;
	geometry_msgs::Quaternion newOrientation;
	
	//zeitdifferenz in sekunden
	double deltaT = ros::Time::now().toSec() - lastOdom.header.stamp.toSec();
	
	//neue geschwindigkeit aus beschleunigung
	newLinVeloX=lastOdom.twist.twist.linear.x + imu->linear_acceleration.x * deltaT;
	newLinVeloY=lastOdom.twist.twist.linear.y + imu->linear_acceleration.y * deltaT;
	//newLinVeloZ = 0
	
	//neue position aus geschwindigkeit
	newX = lastOdom.pose.pose.position.x + deltaT * newLinVeloX; //geschwindigkeit in richtiger einheit?!
	newY = lastOdom.pose.pose.position.y + deltaT * newLinVeloY;
	
	//neue orientierung -> rotation um z achse, wir brauchen orientierung der z achse...(als winkel) dann mit winkel geschw verrechnen
	geometry_msgs::Quaternion test = lastOdom.pose.pose.orientation;
	//########## warum geht eine multiplikation nicht?!?!?!
	//newOrientation = lastOdom.pose.pose.orientation * tf::createQuaternionMsgFromYaw(imu->angular_velocity.z * deltaT); //Neues Quaternion, repraesentiert neue rotierung seit letztem Zeitschritt, Quaternionen_gesamt = Quaternionen_rotation1 * Quaternion_rotation2 NICHT: " + "
	newOrientation = tf::createQuaternionMsgFromYaw(imu->angular_velocity.z * deltaT); // eigentlich ...=test * tf::create....
	// eine drehung um x und y erfolgt nicht! 
	
	//Speichern der neuen Ergebnisse
	saveOdometry(&lastOdom, ros::Time::now(), newX, newY, newOrientation);

	//first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans; // odom_trans wird fuer transformation deklariert
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link"; //warum?
	
	//braucht tf die absoluten oder relativen werte? falls relativ, korriegieren!
	odom_trans.transform.translation.x = newX;
	odom_trans.transform.translation.y = newY;
	odom_trans.transform.translation.z = 0;
	
	odom_trans.transform.rotation = newOrientation; //neue orientierung
	
	
    //send the transform
    odom_broadcaster->sendTransform(odom_trans);


	
	//Publishen der Odometry Msg: odom=Header,Position, Orientierung, Twist(Geschwindigkeit linear und angular)
    nav_msgs::Odometry odom; //deklaration der zu sendenden msg
    //-----start:msg mit inhalt fuellen-------------
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
	
	//set the pose
    odom.pose.pose.position.x = newX;
    odom.pose.pose.position.y = newY;
    odom.pose.pose.position.z = 0;

    odom.pose.pose.orientation = newOrientation;

    // angular velocity
    odom.twist.twist.angular = imu->angular_velocity; //Winkelgeschwindigkeit wird direkt gemessen
    
    //lineare geschwindigkeit
    odom.twist.twist.linear.x=newLinVeloX;
    odom.twist.twist.linear.y=newLinVeloY;
    odom.twist.twist.linear.z=0;
	//-----end:msg mit inhalt fuellen-------------
    
    //publish the message
    lastOdom = odom;
    odom_pub.publish(odom);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "imu_publisher"); // Name des Nodes

  ros::NodeHandle n;
  
  saveOdometry(&lastOdom, ros::Time::now(), 0, 0, tf::createQuaternionMsgFromYaw(0));
  ROS_INFO("initalized Odometry");
  
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50); //Publisher odom_pub publisht den Messagetyp nav_msgs/Odometry auf Topic /odom mit einem Buffer von 50 nachrichten
  
  odom_broadcaster = new tf::TransformBroadcaster(); //Initialisiere Transform
  
  ros::Subscriber imu_sub = n.subscribe("/sensor_msgs/Imu", 1, imuCallBack);  //Verknuepfe imuCallback

   ros::spin();
}
