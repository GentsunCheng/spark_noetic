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
#include <swiftpro/buzzer.h>

serial::Serial _serial;				// serial object
swiftpro::SwiftproState pos;

/*
 * Description: callback when receive data from position_write_topic
 * Inputs: 		msg(float)			3 cartesian coordinates: x, y, z(mm)
 * Outputs:		Gcode				send gcode to control swift pro
 */
void position_write_callback(const swiftpro::position& msg)
{
    std::string Gcode;
    std_msgs::String result;
    std::string x = std::to_string(msg.x);
    std::string y = std::to_string(msg.y);
    std::string z = std::to_string(msg.z);
    std::string speed;
    
    if (msg.speed) {
		speed = std::to_string(msg.speed);
    } else {
        speed = "1000";
    }

    Gcode = "G0 X" + x + " Y" + y + " Z" + z + " F" + speed + "\r\n";
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
	std::string m1 = std::to_string(msg.angle1st);

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
	std::string m2 = std::to_string(msg.angle2nd);

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
	std::string m3 = std::to_string(msg.angle3rd);

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
	std::string m4 = std::to_string(msg.angle4th);

	Gcode = (std::string)"G2202 N3 V" + m4 + "F500\r\n";
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
	std::string status = std::to_string(msg.status);

	Gcode = (std::string)"M2231 V" + status + "\r\n";

	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}

/*
 * Description: callback when receive data from buzzer_topic
 * Inputs: 		msg(buzzer)			buzzer time and frequency
 * Outputs:		Gcode				send gcode to control swift pro
*/
void buzzer_callback(const swiftpro::buzzer& buzzer)
{
	std::string Gcode = "";
	std_msgs::String result;
	std::string time = std::to_string((int(buzzer.time)) * 1000);
	std::string frequent = std::to_string(buzzer.frequent);

	Gcode = (std::string)"M2210 F" + frequent + " T" + time + "\r\n";

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
	std::string Gcode = "";

	ros::Subscriber sub_position = nh.subscribe("position_write_topic", 10, position_write_callback);
	ros::Subscriber sub_status = nh.subscribe("swiftpro_status_topic", 10, swiftpro_status_callback);
	ros::Subscriber sub_gripper = nh.subscribe("gripper_topic", 10, gripper_callback);
	ros::Subscriber sub_pump = nh.subscribe("pump_topic", 10, pump_callback);
	ros::Subscriber sub_buzzer = nh.subscribe("buzzer_topic", 10, buzzer_callback);
    ros::Subscriber sub_angle1st = nh.subscribe("angle1st_topic", 10, angle1st_callback);
    ros::Subscriber sub_angle2nd = nh.subscribe("angle2nd_topic", 10, angle2nd_callback);
    ros::Subscriber sub_angle3rd = nh.subscribe("angle3rd_topic", 10, angle3rd_callback);
    ros::Subscriber sub_angle4th = nh.subscribe("angle4th_topic", 10, angle4th_callback);
	ros::Publisher 	 pub = nh.advertise<swiftpro::SwiftproState>("SwiftproState_topic", 10);
	ros::Rate loop_rate(200);

	try
	{
		_serial.setPort("/dev/ttyACM0");
		_serial.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		_serial.setTimeout(to);
		_serial.open();
		Gcode = (std::string)"M2234 V0\n\r\n";
		_serial.write(Gcode.c_str());
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


