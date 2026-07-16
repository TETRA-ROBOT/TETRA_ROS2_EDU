#include "helper.h"

#if defined (ROS1)
#include <ros1/kanavi_node.h>

// Entry point for this module
int main(int argc, char** argv)
{
    if (checkHelpOptionROS1(argc, argv)) 
	{
        return 0;
    }

	ros::init(argc, argv, "r2");
	KanaviNode node("r2", argc, argv);
	node.Run();

	return 0;
}

#elif defined (ROS2)

#include <ros2/kanavi_node.h>

// Entry point for this module
int main(int argc, char** argv)
{
    if (checkHelpOptionROS2(argc, argv)) 
	{
        return 0;
    }

	// init ROS2
	rclcpp::init(argc, argv);

	// generate node
	auto node = std::make_shared<KanaviNode>("r2", argc, argv);

	// start node
	rclcpp::spin(node);

	// exit node
	rclcpp::shutdown();

	return 0;
}
#endif
