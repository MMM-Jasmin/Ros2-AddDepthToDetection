// SYSTEM
#include <atomic>
#include <csignal>
#include <cstdio>
// ROS2
#include <rclcpp/rclcpp.hpp>

#include "add_depth_to_detection_node.hpp"

/**
 * @brief Exit request flag.
 */
//static std::atomic<bool> exit_request(false);

/**
 * @brief Check if given command line argument exists.
 * @param begin Pointer to the beginning of the argument array
 * @param end Pointer to the end of the argument array
 * @param argument The command line argument to check for
 * @return True if command line argument exists, false otherwise
 */
bool cmdArgExists(char** begin, char** end, const std::string& argument)
{
	return std::find(begin, end, argument) != end;
}

/**
 * @brief Get value of given command line argument.
 * @param begin Pointer to the beginning of the argument array
 * @param end Pointer to the end of the argument array
 * @param argument Command line argument to get the value for
 * @return Pointer to the command line argument value
 */
char* getCmdArg(char** begin, char** end, const std::string& argument)
{
	char** itr = std::find(begin, end, argument);
	if (itr != end && ++itr != end)
		return *itr;

	return nullptr;
}

/**
 * @brief Handler for received process signals.
 * @param signum Code of the received signal
 */
void signalHandler(int signum)
{
	std::cout << "+==========[ Signal " << signum << " Abort ]==========+" << std::endl;
	//exit_request.store(true);
}

/**
 * @brief Main function.
 * @param argc Number of command line arguments
 * @param argv Given command line arguments
 * @return EXIT_SUCCESS (0) on clean exit, EXIT_FAILURE (1) on error state
 */
int main(int argc, char** argv)
{
	signal(SIGINT, signalHandler);

	std::cout << "+==========[ Add Depth Node ]==========+" << std::endl;
	rclcpp::init(argc, argv);

	std::shared_ptr<AddDepthToDetectionNode> image_node = std::make_shared<AddDepthToDetectionNode>();
	//image_node->setExitSignal(&exit_request);
	image_node->init();

	//rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2, false);
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(image_node);

	executor.spin();
	executor.cancel();
	rclcpp::shutdown();

	std::cout << "+==========[ Shutdown ]==========+" << std::endl;
	return EXIT_SUCCESS;
}