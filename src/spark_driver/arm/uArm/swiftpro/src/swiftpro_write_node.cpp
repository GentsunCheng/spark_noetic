/*
 * Software License Agreement (BSD License)
 * Copyright (c) 2017, UFactory, Inc.
 * All rights reserved.
 * Author: Roger Cui  <roger@ufactory.cc>
 */
#include <string>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <swiftpro/SwiftproState.h>
#include <swiftpro/status.h>
#include <swiftpro/position.h>
#include <swiftpro/angle4th.h>
#include <swiftpro/angle3rd.h>
#include <swiftpro/angle2nd.h>
#include <swiftpro/angle1st.h>

serial::Serial _serial;				// serial object
swiftpro::SwiftproState pos;

/*
 * Description: callback when receive data from position_write_topic
 * Inputs: 		msg(float)			3 cartesian coordinates: x, y, z(mm)
 * Outputs:		Gcode				send gcode to control swift pro
 */
void position_write_callback(const swiftpro::position& msg)
{
	std::string Gcode = "";
	std_msgs::String result;
	char x[10];
	char y[10];
	char z[10];

	pos.x = msg.x;
	pos.y = msg.y;
	pos.z = msg.z;
	sprintf(x, "%.2f", msg.x);
	sprintf(y, "%.2f", msg.y);
	sprintf(z, "%.2f", msg.z);
	Gcode = (std::string)"G0 X" + x + " Y" + y + " Z" + z + " F20000" + "\r\n";
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}


/*
 * Description: callback when receive data from angle1st_topic
 * Inputs: 		msg(float)			angle of 1st motor(degree)
 * Outputs:		Gcode				send gcode to control swift pro
 */
void angle1st_callback(const swiftpro::angle1st& msg)
{
	std::string Gcode = "";
	std_msgs::String result;
	char m1[10];

	pos.motor_angle1 = msg.angle1st;
	sprintf(m1, "%.2f", msg.angle1st);
	Gcode = (std::string)"G2202 N0 V" + m1 + "F100\r\n";
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}


/*
 * Description: callback when receive data from angle2nd_topic
 * Inputs: 		msg(float)			angle of 2nd motor(degree)
 * Outputs:		Gcode				send gcode to control swift pro
 */
void angle2nd_callback(const swiftpro::angle2nd& msg)
{
	std::string Gcode = "";
	std_msgs::String result;
	char m2[10];

	pos.motor_angle2 = msg.angle2nd;
	sprintf(m2, "%.2f", msg.angle2nd);
	Gcode = (std::string)"G2202 N1 V" + m2 + "F100\r\n";
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}


/*
 * Description: callback when receive data from angle3rd_topic
 * Inputs: 		msg(float)			angle of 3rd motor(degree)
 * Outputs:		Gcode				send gcode to control swift pro
 */
void angle3rd_callback(const swiftpro::angle3rd& msg)
{
	std::string Gcode = "";
	std_msgs::String result;
	char m3[10];

	pos.motor_angle3 = msg.angle3rd;
	sprintf(m3, "%.2f", msg.angle3rd);
	Gcode = (std::string)"G2202 N2 V" + m3 + "F100\r\n";
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}


/*
 * Description: callback when receive data from angle4th_topic
 * Inputs: 		msg(float)			angle of 4th motor(degree)
 * Outputs:		Gcode				send gcode to control swift pro
 */
void angle4th_callback(const swiftpro::angle4th& msg)
{
	std::string Gcode = "";
	std_msgs::String result;
	char m4[10];

	pos.motor_angle4 = msg.angle4th;
	sprintf(m4, "%.2f", msg.angle4th);
	Gcode = (std::string)"G2202 N3 V" + m4 + "F100\r\n";
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}


/*
 * Description: callback when receive data from swiftpro_status_topic
 * Inputs: 		msg(uint8)			status of gripper: attach if 1; detach if 0
 * Outputs:		Gcode				send gcode to control swift pro
 */
void swiftpro_status_callback(const swiftpro::status& msg)
{
	std::string Gcode = "";
	std_msgs::String result;

	if (msg.status == 1)
		Gcode = (std::string)"M17\r\n";   // attach
	else if (msg.status == 0)
		Gcode = (std::string)"M2019\r\n";
	else
	{
		ROS_INFO("Error:Wrong swiftpro status input");
		return;
	}

	pos.swiftpro_status = msg.status;
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}


/*
 * Description: callback when receive data from gripper_topic
 * Inputs: 		msg(uint8)			status of gripper: work if 1; otherwise 0
 * Outputs:		Gcode				send gcode to control swift pro
 */
void gripper_callback(const swiftpro::status& msg)
{
	std::string Gcode = "";
	std_msgs::String result;

	if (msg.status == 1)
		Gcode = (std::string)"M2232 V1" + "\r\n";
	else if (msg.status == 0)
		Gcode = (std::string)"M2232 V0" + "\r\n";
	else
	{
		ROS_INFO("Error:Wrong gripper status input");
		return;
	}

	pos.gripper = msg.status;
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}


/*
 * Description: callback when receive data from pump_topic
 * Inputs: 		msg(uint8)			status of pump: work if 1; otherwise 0
 * Outputs:		Gcode				send gcode to control swift pro
 */
void pump_callback(const swiftpro::status& msg)
{
	std::string Gcode = "";
	std_msgs::String result;

	if (msg.status == 1)
		Gcode = (std::string)"M2231 V1" + "\r\n";
	else if (msg.status == 0)
		Gcode = (std::string)"M2231 V0" + "\r\n";
    else
	{
		ROS_INFO("Error:Wrong pump status input");
		return;
	}

	pos.pump = msg.status;
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}


/*
 * Node name:
 *	 swiftpro_write_node
 *
 * Topic publish: (rate = 200Hz, queue size = 10)
 *	 swiftpro_state_topic
 *
 * Topic subscribe: (queue size = 10)
 *	 position_write_topic
 *	 swiftpro_status_topic
 *	 angle4th_topic
 *	 gripper_topic
 *	 pump_topic
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "swiftpro_write_node");
	ros::NodeHandle nh;
	swiftpro::SwiftproState swiftpro_state;

	ros::Subscriber sub1 = nh.subscribe("position_write_topic", 10, position_write_callback);
	ros::Subscriber sub2 = nh.subscribe("swiftpro_status_topic", 10, swiftpro_status_callback);
	ros::Subscriber sub3 = nh.subscribe("gripper_topic", 10, gripper_callback);
	ros::Subscriber sub4 = nh.subscribe("pump_topic", 10, pump_callback);
    ros::Subscriber sub6 = nh.subscribe("angle1st_topic", 10, angle1st_callback);
    ros::Subscriber sub7 = nh.subscribe("angle2nd_topic", 10, angle2nd_callback);
    ros::Subscriber sub8 = nh.subscribe("angle3rd_topic", 10, angle3rd_callback);
    ros::Subscriber sub9 = nh.subscribe("angle4th_topic", 10, angle4th_callback);
	ros::Publisher 	 pub = nh.advertise<swiftpro::SwiftproState>("SwiftproState_topic", 10);
	ros::Rate loop_rate(200);

	try
	{
		_serial.setPort("/dev/ttyACM0");
		_serial.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		_serial.setTimeout(to);
		_serial.open();
		ROS_INFO_STREAM("Port has been open successfully");
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port");
		return -1;
	}

	if (_serial.isOpen())
	{
		ros::Duration(3.5).sleep();				// wait 3.5s
		_serial.write("M2120 V0\r\n");			// stop report position
		ros::Duration(0.1).sleep();				// wait 0.1s
		_serial.write("M17\r\n");				// attach
		ros::Duration(0.1).sleep();				// wait 0.1s
		ROS_INFO_STREAM("Attach and wait for commands");
	}

	while (ros::ok())
	{
		pub.publish(pos);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


