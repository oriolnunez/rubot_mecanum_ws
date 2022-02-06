/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;
double x = 1.0;
double y = 0.0;
double theta = 1.57;
double vx = 0.0;
double w = 0.0;
char base_link[] = "/base_link";
char odom_link[] = "/odom";

void messageCb( const geometry_msgs::Twist& cmd_msg) {
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  nh.loginfo("cmd_vel received!");
  vx=cmd_msg.linear.x;
  w=cmd_msg.angular.z;
  nh.loginfo("vel published!:");
  //nh.loginfo(vx);
  
}

ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", messageCb );

sensor_msgs::Range range_msg;
ros::Publisher pub_range( "range_data", &range_msg);
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;


void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  broadcaster.init(nh);
  nh.advertise(odom_pub);
  nh.advertise(pub_range);
  nh.subscribe(sub_vel);
}

void loop()
{
  // drive in a circle
  double dx = 0.2;
  double dtheta = 0.18;
  x += cos(theta)*dx*0.1;
  y += sin(theta)*dx*0.1;
  theta += dtheta*0.1;
  if(theta > 3.14)
    theta=-3.14;
    
  // tf odom->base_link
  t.header.frame_id = odom_link;
  t.child_frame_id = base_link;
  
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  
  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();
  
  broadcaster.sendTransform(t);
  
  // Odom Publishing
  odom.header.stamp = nh.now();;
  odom.header.frame_id = "odom";
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation =tf::createQuaternionFromYaw(theta);;

  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = 3;
  odom.twist.twist.linear.y = 4;
  odom.twist.twist.angular.z = 5;
  //odom_pub.publish(&odom);

  // Range Publishing
  range_msg.range = 3;
  //pub_range.publish(&range_msg);
  
  nh.spinOnce();
  delay(50);
}
