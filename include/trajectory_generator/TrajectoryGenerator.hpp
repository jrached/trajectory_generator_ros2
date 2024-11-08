/**
 * @file TrajectoryGenerator.hpp
 * @brief Trajectory Generator class
 * @author Aleix Paris
 * @date 2020-01-08
 */

#pragma once

#include <vector>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "trajectory_generator/trajectories/Trajectory.hpp"

// ROS
#include <ros/ros.h>
#include <snapstack_msgs/msg/goal.h>
// #include <snapstack_msgs/msg/QuadFlightMode.h>
#include <snapstack_msgs/msg/quad_flight_mode.h>
#include <snapstack_msgs/msg/state.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/quaternion.h>

namespace trajectory_generator {

/*!
 * Main class for the node to handle the ROS interfacing.
 */

// Inherit rclcpp Node class
class TrajectoryGenerator : public rclcpp::Node
{
public:
    /*!
   * Constructor.
   */
   TrajectoryGenerator();

    /*!
   * Destructor.
   */
    virtual ~TrajectoryGenerator();

private:
    /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
    bool readParameters();

    void modeCB(const snapstack_msgs::msg::QuadFlightMode& msg);
    void stateCB(const snapstack_msgs::msg::State& msg);
    void pubCB(const rclcpp::TimerEvent& event);

    void resetGoal();

    // Utils
    static snapstack_msgs::msg::Goal simpleInterpolation(const snapstack_msgs::msg::Goal& current,
                                                        const geometry_msgs::msg::Vector3& dest_pos,
                                                        double dest_yaw, double vel, double vel_yaw,
                                                        double dist_thresh, double yaw_thresh, double dt, bool& finished);
    static snapstack_msgs::msg::Goal simpleInterpolation(const snapstack_msgs::msg::Goal& current,
                                                        const snapstack_msgs::msg::Goal& dest_pos,
                                                        double dest_yaw, double vel, double vel_yaw,
                                                        double dist_thresh, double yaw_thresh, double dt, bool& finished);
    static double quat2yaw(const geometry_msgs::msg::Quaternion& q);
    static double saturate(double val, double low, double high);
    static double wrap(double val);

    // ROS
    rclcpp::Subscription<snapstack_msgs::msg::QuadFlightMode>::SharedPtr subs_mode_;  // "flightmode" Subscription
    rclcpp::Subscription<snapstack_msgs::msg::State>::SharedPtr subs_state_;  // "state" Subscription
    rclcpp::Publisher<snapstack_msgs::msg:Goal>::SharedPtr pub_goal_;  // "goal" publisher
    rclcpp::TimerBase::SharedPtr pub_timer_;  // timer for pub_goal

    // Trajectory
    std::unique_ptr<Trajectory> traj_;

    std::string veh_name_;  // vehicle name
    enum FlightMode {GROUND,
                     TAKING_OFF,
                     HOVERING,
                     INIT_POS_TRAJ,
                     TRAJ_FOLLOWING,
                     LANDING,
                     INIT_POS
                    };
    FlightMode flight_mode_;
    geometry_msgs::msg::Pose pose_;
    snapstack_msgs::msg::Goal goal_;

    double alt_;  // altitude in m where to take off, and set the traj alt to this too
    double dt_;  // goal publication period [s], and set the traj dt to this too

    std::vector<snapstack_msgs::msg::Goal> traj_goals_;  // vector of trajectory goals currently being followed
    std::vector<snapstack_msgs::msg::Goal> traj_goals_full_;  // goals for all the maneuver: circles with v = v_goals_
    std::unordered_map<int, std::string> index_msgs_;
    std::unordered_map<int, std::string> index_msgs_full_;
    int pub_index_;  // current index in traj_goals_ vector

    double vel_initpos_, vel_take_, vel_land_fast_, vel_land_slow_, vel_yaw_;  // interpolation vels
    double dist_thresh_, yaw_thresh_;
    double margin_takeoff_outside_bounds_;
    double xmin_, xmax_, ymin_, ymax_, zmin_, zmax_;  // safety bouds

    geometry_msgs::msg::Vector3 init_pos_;  // pos where we took off and where we will land
    // z component is the initial altitude of the robot before take off, "ground level"
};

} /* namespace */
