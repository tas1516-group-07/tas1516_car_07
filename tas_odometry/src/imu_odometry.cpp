//Roman, TAS-GRP7, V1
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/LinkStates.h>



ros::Publisher odom_pub;
tf::TransformBroadcaster* odom_broadcaster;
//nav_msgs::Odometry lastOdom; //Speichern der letzten Odometry

//Speichern letzte Odometrie
double dblPosX = 0.0;
double dblPosY = 0.0;

double dblAngZ = 0.0;

double dblLinVeloX = 0.0;
double dblLinVeloY = 0.0;

ros::Time timeLastTime = ros::Time::now();

void imuCallBack(const sensor_msgs::Imu::ConstPtr& imu) //Erhaelt Msg vom Typ sensor_msgs/Imu von Topic /imu
{
	/* imuCallBack
	 * --------------
	 * wird immmer dann aufgerufen, wenn eine neue Msg der Imu ankommnt
	 */
	 
	 /*
	  * Diskussion:
	  * - frequenz IMU schnell genug?
	  * - spin() schneller als imu frequenz?
	  * - in welche Richtung liefert IMU die Daten? evlt orientierug verrechnen?
	  * 
	  *     double dt = (current_time - last_time).toSec();
			double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
			double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
			double delta_th = vth * dt;
	  */
	
	ROS_INFO("received IMU message");
	double dblDeltaT;
	geometry_msgs::Quaternion newOrientation;
	
	//zeitdifferenz in sekunden
	ros::Time timeCurrentTime = ros::Time::now();
	dblDeltaT = timeCurrentTime.toSec() - timeLastTime.toSec();
	
	//neue geschwindigkeit aus beschleunigung
	dblLinVeloX = dblLinVeloX + imu->linear_acceleration.x * dblDeltaT;
	dblLinVeloY = dblLinVeloY + imu->linear_acceleration.y * dblDeltaT;
	//dblLinVeloZ = 0
	
	//neue position aus geschwindigkeit
	dblPosX = dblPosX + dblDeltaT * dblLinVeloX;
	dblPosY = dblPosY + dblDeltaT * dblLinVeloY;
	
	//neue orientierung
	dblAngZ = dblAngZ + imu->angular_velocity.z * dblDeltaT;
	geometry_msgs::Quaternion quatNewOrientation = tf::createQuaternionMsgFromYaw(dblAngZ);
	// eine drehung um x und y erfolgt nicht! 
	
	//Publishen der Transform ueber tf
    geometry_msgs::TransformStamped odom_trans; // odom_trans wird fuer transformation deklariert
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
	
	//Odometrie-Daten fuer tf
	odom_trans.transform.translation.x = dblPosX;
	odom_trans.transform.translation.y = dblPosY;
	odom_trans.transform.translation.z = 0;
	odom_trans.transform.rotation = quatNewOrientation; //neue orientierung
	
    //Sende Transformation
    odom_broadcaster->sendTransform(odom_trans);
	
	//Publishen der Odometry Msg: Header,Position, Orientierung, Twist(Geschwindigkeit linear und angular)
    nav_msgs::Odometry odom; //deklaration der zu sendenden msg
    
    //Daten
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
	
	//Setze Pose
    odom.pose.pose.position.x = dblPosX;
    odom.pose.pose.position.y = dblPosY;
    odom.pose.pose.position.z = 0;

    odom.pose.pose.orientation = quatNewOrientation;

    // Winkelgeschwindigkeit
    odom.twist.twist.angular = imu->angular_velocity; //Winkelgeschwindigkeit wird direkt gemessen
    
    //Lineare Geschwindigkeit
    odom.twist.twist.linear.x=dblLinVeloX;
    odom.twist.twist.linear.y=dblLinVeloY;
    odom.twist.twist.linear.z=0;
    
    //Publish Msg
    odom_pub.publish(odom);
    
    //Zeit speichern
    timeLastTime = timeCurrentTime;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "imu_publisher"); // Name des Nodes

  ros::NodeHandle n;
  
  ROS_INFO("Odometry initalized"); 

  //Wurde durch initialisierung ersetzt - saveOdometry(&lastOdom, ros::Time::now(), 0, 0, tf::createQuaternionMsgFromYaw(0));
  
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50); //Publisher odom_pub publisht den Messagetyp nav_msgs/Odometry auf Topic /odom mit einem Buffer von 50 nachrichten
  
  odom_broadcaster = new tf::TransformBroadcaster(); //Initialisiere Transform
  
  ros::Subscriber imu_sub = n.subscribe("/sensor_msgs/Imu", 1, imuCallBack);  //Verknuepfe imuCallback

  ros::spin(); //checke, ob neue msg da ist
}
