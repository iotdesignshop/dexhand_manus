/// @file dexhand_manus.cpp
/// @brief This node is an intermediary between the manus_ros2 node and the DexHand. It takes the joint data
/// from the Manus Gloves and converts it into joint state messages consumed by the DexHand.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sys/time.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <fstream>
#include <iostream>
#include <thread>
#include <map>


// Needed for isKeyPressed
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

using namespace std::chrono_literals;
using namespace std;

static const int NUM_FINGERS_ON_HAND = 5;

// Global for now to pass joint data between the Manus SDK and ROS2
float g_euler_joint[24][3] = {0.0};

bool KeyDown();

class JointLimits
{
	public:
		JointLimits(float min, float max) : min(min), max(max) {}
		float apply(float value)
		{
			if (value < min)
				return min;
			if (value > max)
				return max;
			return value;
		}

	private:
		float min;
		float max;
};

// Ultimately, we would want to load these Joint Limits from the URDF or a configuration file
// but for now, we will include these in a table here. 
std::map<std::string, JointLimits> joint_limits = {
	{"wrist_pitch_lower", JointLimits(-0.52, 0.52)},
	{"wrist_pitch_upper", JointLimits(-0.349, 0.349)},
	{"wrist_yaw", JointLimits(-0.436, 0.436)},
	{"index_yaw", JointLimits(-0.349, 0.349)},
	{"middle_yaw", JointLimits(-0.349, 0.349)},
	{"ring_yaw", JointLimits(-0.349, 0.349)},
	{"pinky_yaw", JointLimits(-0.349, 0.349)},
	{"index_pitch", JointLimits(0, 1.05)},
	{"index_knuckle", JointLimits(0, 0.785)},
	{"index_tip", JointLimits(0, 0.785)},
	{"middle_pitch", JointLimits(0, 1.05)},
	{"middle_knuckle", JointLimits(0, 0.785)},
	{"middle_tip", JointLimits(0, 0.785)},
	{"ring_pitch", JointLimits(0, 1.05)},
	{"ring_knuckle", JointLimits(0, 0.785)},
	{"ring_tip", JointLimits(0, 0.785)},
	{"pinky_pitch", JointLimits(0, 1.05)},
	{"pinky_knuckle", JointLimits(0, 0.785)},
	{"pinky_tip", JointLimits(0, 0.785)},
	{"thumb_yaw", JointLimits(0, 0.872)},
	{"thumb_roll", JointLimits(0, 0.349)},
	{"thumb_pitch", JointLimits(0, 1.047)},
	{"thumb_knuckle", JointLimits(0, 0.785)},
	{"thumb_tip", JointLimits(0, 0.785)}
};

/// @brief ROS2 publisher class for the manus_ros2 node
class DexHandManus : public rclcpp::Node
{
public:
	DexHandManus() : Node("dexhand_manus")
	{
		RCLCPP_INFO(this->get_logger(), "DexhandManus node started");

		publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

		manus_left_listener_ = this->create_subscription<geometry_msgs::msg::PoseArray>("manus_left", 10, std::bind(&DexHandManus::manus_left_callback, this, std::placeholders::_1));
		manus_right_listener_ = this->create_subscription<geometry_msgs::msg::PoseArray>("manus_right", 10, std::bind(&DexHandManus::manus_right_callback, this, std::placeholders::_1));

		command_subscriber_ = this->create_subscription<std_msgs::msg::String>(
			"dexhand_manus_cmd", 10, std::bind(&DexHandManus::command_callback, this, std::placeholders::_1));


		// Start a timer to publish default joint states until we get a message
		default_timer_ = this->create_wall_timer(10ms, std::bind(&DexHandManus::publish_joint_states, this));

		
    }

	// print_joint - prints global joint data from Manus Hand after transform
	void print_joint(int joint)
	{
    RCLCPP_INFO_STREAM(this->get_logger(), 
		  std::fixed << std::setprecision(5) << "Joint: " << joint << " " 
        << (g_euler_joint[joint][0] >= 0 ? " " : "") << g_euler_joint[joint][0] << " " 
        << (g_euler_joint[joint][1] >= 0 ? " " : "") << g_euler_joint[joint][1] << " " 
        << (g_euler_joint[joint][2] >= 0 ? " " : "") << g_euler_joint[joint][2] << " "
			  << "\n");
	}

private:

	// cancels the default timer
	void cancel_default_timer()
	{
		if (default_timer_ != nullptr) {
			default_timer_->cancel();
			default_timer_ = nullptr;
		}
		
	}

	// manus_left_callback - callback for the left manus glove
	void manus_left_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
	{
		cancel_default_timer();
		processPose(msg);
		RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Receiving Manus Left Data");
	}

	// manus_right_callback - callback for the right manus glove
	void manus_right_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
	{
		cancel_default_timer();

		processPose(msg);
		RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Receiving Manus Right Data");
	}

	// command_callback - callback for the command subscriber
	void command_callback(const std_msgs::msg::String::SharedPtr msg)
	{
		if (msg->data == "reset_origin")
		{
			// Reset the wrist position to center
			initialized = false;
			RCLCPP_INFO(this->get_logger(), "Resetting wrist position to center");
		}
		else
		{
			RCLCPP_INFO_STREAM(this->get_logger(), "Unknown command received:" << msg->data << "\n");
		}
	}

	// Publishes the joint states to the ROS2 network
	void publish_joint_states()
	{
		auto joint_state = sensor_msgs::msg::JointState();

		joint_state.name.push_back("wrist_pitch_lower");
		joint_state.name.push_back("wrist_pitch_upper");
		joint_state.name.push_back("wrist_yaw");
		joint_state.name.push_back("index_yaw");
		joint_state.name.push_back("middle_yaw");
		joint_state.name.push_back("ring_yaw");
		joint_state.name.push_back("pinky_yaw");
		joint_state.name.push_back("index_pitch");
		joint_state.name.push_back("index_knuckle");
		joint_state.name.push_back("index_tip");
		joint_state.name.push_back("middle_pitch");
		joint_state.name.push_back("middle_knuckle");
		joint_state.name.push_back("middle_tip");
		joint_state.name.push_back("ring_pitch");
		joint_state.name.push_back("ring_knuckle");
		joint_state.name.push_back("ring_tip");
		joint_state.name.push_back("pinky_pitch");
		joint_state.name.push_back("pinky_knuckle");
		joint_state.name.push_back("pinky_tip");
		joint_state.name.push_back("thumb_yaw");
		joint_state.name.push_back("thumb_roll");
		joint_state.name.push_back("thumb_pitch");
		joint_state.name.push_back("thumb_knuckle");
		joint_state.name.push_back("thumb_tip");

		//const std::string t_FingerNames[NUM_FINGERS_ON_HAND] = {"[thumb] ", "[index] ", "[middle]", "[ring]  ", "[pinky] "};
		//const std::string t_FingerJointNames[NUM_FINGERS_ON_HAND] = {"mcp", "pip", "dip"};
		//const std::string t_ThumbJointNames[NUM_FINGERS_ON_HAND] = {"cmc", "mcp", "ip "};

		// Wrist pitch is spread across two joints

		joint_state.position.push_back(joint_limits.at("wrist_pitch_lower").apply(-g_euler_joint[0][1]/2)); // "wrist_pitch_lower"
		joint_state.position.push_back(joint_limits.at("wrist_pitch_upper").apply(-g_euler_joint[0][1]/2)); // "wrist_pitch_upper"

		joint_state.position.push_back(joint_limits.at("wrist_yaw").apply(g_euler_joint[0][2]));  // "wrist_yaw"
		joint_state.position.push_back(joint_limits.at("index_yaw").apply(g_euler_joint[5][2]));  // "index_yaw"
		joint_state.position.push_back(joint_limits.at("middle_yaw").apply(g_euler_joint[9][2]));  // "middle_yaw"
		joint_state.position.push_back(joint_limits.at("ring_yaw").apply(g_euler_joint[13][2])); // "ring_yaw"
		joint_state.position.push_back(joint_limits.at("pinky_yaw").apply(g_euler_joint[17][2])); // "pinky_yaw"
		joint_state.position.push_back(joint_limits.at("index_pitch").apply(g_euler_joint[5][1]));  // "index_pitch"
		joint_state.position.push_back(joint_limits.at("index_knuckle").apply(g_euler_joint[6][1]));  // "index_knuckle"
		joint_state.position.push_back(joint_limits.at("index_tip").apply(g_euler_joint[7][1]));  // "index_tip"
		joint_state.position.push_back(joint_limits.at("middle_pitch").apply(g_euler_joint[9][1]));  // "middle_pitch"
		joint_state.position.push_back(joint_limits.at("middle_knuckle").apply(g_euler_joint[10][1])); // "middle_knuckle"
		joint_state.position.push_back(joint_limits.at("middle_tip").apply(g_euler_joint[11][1])); // "middle_tip"
		joint_state.position.push_back(joint_limits.at("ring_pitch").apply(g_euler_joint[13][1])); // "ring_pitch"
		joint_state.position.push_back(joint_limits.at("ring_knuckle").apply(g_euler_joint[14][1])); // "ring_knuckle"
		joint_state.position.push_back(joint_limits.at("ring_tip").apply(g_euler_joint[15][1])); // "ring_tip"
		joint_state.position.push_back(joint_limits.at("pinky_pitch").apply(g_euler_joint[17][1])); // "pinky_pitch"
		joint_state.position.push_back(joint_limits.at("pinky_knuckle").apply(g_euler_joint[18][1])); // "pinky_knuckle"
		joint_state.position.push_back(joint_limits.at("pinky_tip").apply(g_euler_joint[19][1])); // "pinky_tip"
		joint_state.position.push_back(joint_limits.at("thumb_yaw").apply(g_euler_joint[1][1]*2.0));  // "thumb_yaw"
		joint_state.position.push_back(joint_limits.at("thumb_roll").apply(-g_euler_joint[1][0]));  // "thumb_roll"
		joint_state.position.push_back(joint_limits.at("thumb_pitch").apply(-g_euler_joint[1][2]*2.0));  // "thumb_pitch"
		joint_state.position.push_back(joint_limits.at("thumb_knuckle").apply(-g_euler_joint[2][2])); // "thumb_knuckle"
		joint_state.position.push_back(joint_limits.at("thumb_tip").apply(-g_euler_joint[3][2])); // "thumb_tip"

		// Set timestamp to current ROS time
		joint_state.header.stamp = this->now();

		// Command the robot to move
		publisher_->publish(joint_state);
	}

	void processPose(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
		double roll, pitch, yaw;
		
		// Initialize with identity quaternion
		static tf2::Quaternion referenceRotation(1.0, 0.0, 0.0, 0.0);
		
		// Put the skeleton data in the bridge structure
		for (uint32_t i = 0; i < msg->poses.size(); i++) {

			const auto &joint = msg->poses[i];

			// Calculate delta rotation only for the wrist (joint 0)
			if (i == 0) {
				if (!initialized) {
					// Store the current orientation
					referenceRotation = tf2::Quaternion(joint.orientation.x, joint.orientation.y, joint.orientation.z, joint.orientation.w);
					initialized = true;
				}
				else if (KeyDown()) {	
					initialized = false;
				}

				// Calculate the inverse of the reference rotation
				tf2::Quaternion referenceRotationInverse = referenceRotation.inverse();
				
				// Calculate the delta rotation
				tf2::Quaternion newRotationInverse = referenceRotationInverse * tf2::Quaternion(joint.orientation.x, joint.orientation.y, joint.orientation.z, joint.orientation.w);
				
				// Convert the delta rotation to Euler angles
				tf2::Matrix3x3(newRotationInverse).getRPY(roll, pitch, yaw);

				// Assign roll, pitch, and yaw directly to g_dexhand_joint
				g_euler_joint[i][0] = roll;
				g_euler_joint[i][1] = pitch;
				g_euler_joint[i][2] = yaw;
			}
			else {
				// For child joints, perform regular quaternion to Euler angle conversion
				tf2::Quaternion quaternion(joint.orientation.x, joint.orientation.y, joint.orientation.z, joint.orientation.w);
				tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
				
				// Assign roll, pitch, and yaw directly to g_dexhand_joint
				g_euler_joint[i][0] = roll;
				g_euler_joint[i][1] = pitch;
				g_euler_joint[i][2] = yaw;
			}
		}

	#if DEBUG_PRINT
			std::cout << "Node: " << i << " Rotation X, Y, Z, W "
					<< joint.transform.rotation.x << ", "
					<< joint.transform.rotation.y << ", "
					<< joint.transform.rotation.z << ", "
					<< joint.transform.rotation.w << ")" << std::endl;

			// Print Euler angles
			std::cout << "Node: " << i << " Roll, Pitch, Yaw: "
					<< roll << ", " << pitch << ", " << yaw << std::endl;
	#endif

		// Publish the joint states
		publish_joint_states();
	}

	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
	rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr manus_left_listener_;
	rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr manus_right_listener_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_;
	rclcpp::TimerBase::SharedPtr default_timer_;

	bool initialized = false;
};


// Scan for a key press - Used to reset wrist position to center
bool KeyDown()
{
	struct termios oldt, newt;
	int oldf;

	// Get the current terminal settings
	tcgetattr(STDIN_FILENO, &oldt);

	// Save the current terminal settings so we can restore them later
	newt = oldt;

	// Disable canonical mode and echo
	newt.c_lflag &= ~(ICANON | ECHO);

	// Apply the new terminal settings
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);

	// Set the file descriptor for stdin to non-blocking
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	// Try to read a character from stdin
	char ch;
	ssize_t nread = read(STDIN_FILENO, &ch, 1);

	// Restore the old terminal settings
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

	// Restore the file descriptor flags
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	// Check if a character was read
	if (nread == 1)
		return true;
	else
		return false;
}



        


// Main function - Initializes the minimal client and starts the ROS2 node
int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DexHandManus>());
	rclcpp::shutdown();
	return 0;
}
