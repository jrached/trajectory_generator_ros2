/**
 * @file trajectory_generator_node.cpp
 * @brief Trajectory Generator node
 * @author Aleix Paris
 * @date 2020-01-08
 */

#include <rclcpp/rclcpp.hpp>
#include "trajectory_generator_ros2/TrajectoryGenerator.hpp"

#include "TrajectoryGenerator.cpp"
#include "trajectories/Circle.cpp"
#include "trajectories/Line.cpp"
#include "trajectories/Boomerang.cpp"

#include <memory>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // trajectory_generator::TrajectoryGenerator trajectoryGenerator;
    rclcpp::spin(std::make_shared<trajectory_generator::TrajectoryGenerator>());

    //Spoof node 
    // auto node = rclcpp::Node::make_shared("");
    // rclcpp::spin(node);
    return 0;
}
