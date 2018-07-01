/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int8.h>
// %EndTag(INCLUDES)%


double pickupx = 0.0;
double pickupy = -2.0;
double dropoffx = -7.0;
double dropoffy = 0;
uint32_t shape = visualization_msgs::Marker::CUBE;




void location_callback(const std_msgs::Int8::ConstPtr& msg, int* state)
{  
    switch (msg->data) {	

        case 1:
		//marker.action = visualization_msgs::Marker::DELETE;
		//marker_pub.publish(marker);
		//ros::Duration(5).sleep(); 
		*state = 1;
	       
		break;
	case 2:		
		//marker.pose.position.x = dropoffx;
		//marker.pose.position.y = dropoffy;
		//marker.action = visualization_msgs::Marker::ADD;
		//marker_pub.publish(marker);
		//ros::Duration(5).sleep();
                *state = 2;
              
		break;
    }
}




// %Tag(INIT)%
int main( int argc, char** argv )
{
  int state = 0;
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub_locations = n.subscribe<std_msgs::Int8>("/location_state", 10, boost::bind(&location_callback, _1,&state));
  
  double x = 0;
  double y = 0;
  while (ros::ok())
  {
    //ROS_INFO("State: %d", state);
    visualization_msgs::Marker marker;  
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = shape;
    if (state == 0)
    {
	x = pickupx;
        y = pickupy;
    	marker.action = visualization_msgs::Marker::ADD;
    }
    else if (state == 1)
        marker.action = visualization_msgs::Marker::DELETE;
    else if (state == 2)
    {
        x = dropoffx;
        y = dropoffy;
        marker.action = visualization_msgs::Marker::ADD;
    }
   
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;  
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);    
    ros::spinOnce();
    r.sleep();
  }
  
}


