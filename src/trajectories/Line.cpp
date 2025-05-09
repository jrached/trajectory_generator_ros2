/**
 * @file Line.cpp
 * @brief Line class
 * @author Aleix Paris
 * @date 2020-02-19
 */

#include "trajectory_generator_ros2/trajectories/Line.hpp"

#include "snapstack_msgs2/msg/quad_flight_mode.hpp"
#include "snapstack_msgs2/msg/state.hpp"
#include "snapstack_msgs2/msg/goal.hpp"

#include <vector>
#include <unordered_map>
#include <string>
#include <Eigen/Core>

namespace trajectory_generator {
    
Line::Line(double alt, Eigen::Vector3d A, Eigen::Vector3d B,
         std::vector<double> v_goals, double a1, double a3, double dt): alt_(alt),
      A_(A), B_(B), v_goals_(v_goals), a1_(a1), a3_(a3), Trajectory(dt){
        theta_ = atan2(B_.y() - A_.y(), B_.x() - A_.x());
    }
    
Line::~Line() 
{
}

void Line::generateTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                        std::unordered_map<int,std::string>& index_msgs,
                        const rclcpp::Clock::SharedPtr& clock){
    // rclcpp::Time tstart = rclcpp::Time::now();
    rclcpp::Time tstart= clock->now();

    // init pos: A_
    double v = 0;

    goals.push_back(createLineGoal(A_.x(), A_.y(), v, 0, theta_));

    //for(int i = 0; i < v_goals_.size(); ++i){
    double v_goal = v_goals_[0];  // TODO
    index_msgs[goals.size() - 1] = "Line traj: accelerating to " + std::to_string(v_goal) + " m/s";
    // accelerate to the goal velocity
    while(v < v_goal){
        // generate points in the line with *increasing* velocity
        v = std::min(v + a1_*dt_, v_goal);
        goals.push_back(createLineGoal(goals.back().p.x, goals.back().p.y, v, a1_, theta_));
    }

    // we reached v_goal, continue the traj for t2 seconds
    double t2 = get_d2() / v_goal;

    index_msgs[goals.size() - 1] = "Line traj: reached " + std::to_string(v_goal)
        + " m/s, keeping constant v for " + std::to_string(t2) + " s";
    double current_t_traj_ = 0;
    while(current_t_traj_ < t2){
      // generate points with *constant* velocity v == v_goal
      goals.push_back(createLineGoal(goals.back().p.x, goals.back().p.y, v, 0, theta_));
      current_t_traj_ += dt_;
    }
    // now decelerate to 0
    index_msgs[goals.size() - 1] = "Line traj: decelerating to 0 m/s";
    while(v > 0){
      v = std::max(v - a3_*dt_, 0.0);
      goals.push_back(createLineGoal(goals.back().p.x, goals.back().p.y, v, -a3_, theta_));
    }

    // we reached zero velocity
    double thresh = 0.05;
    //RCLCPP_INFO(logger_, "Final point in goals is (" << goals.back().p.x << ", " << goals.back().p.y << ")");
    //for(auto goal : goals){
    //    std::cout << "(" << goal.p.x << ", " << goal.p.y << ")" << std::endl;
    //}
    if(fabs(B_.x() - goals.back().p.x) > thresh or fabs(B_.y() - goals.back().p.y) > thresh){
        RCLCPP_ERROR(logger_, "Error: final point is not B");
        exit(1);
    }
    // Force last goal pos to be equal to B
    goals.back().p.x = B_.x();
    goals.back().p.y = B_.y();

    index_msgs[goals.size() - 1] = "Line traj: stopped";

    // RCLCPP_INFO(logger_, "Time to calculate the traj (s): %f", (rclcpp::Time::now() - tstart).seconds());
    RCLCPP_INFO(logger_, "Time to calculate the traj (s): %f", (clock->now() - tstart).seconds());
    RCLCPP_INFO(logger_, "Goal vector size = %lu", goals.size());
}

snapstack_msgs2::msg::Goal Line::createLineGoal(double last_x, double last_y, double v, double accel, double theta) const{
    // from A to B: theta. From B to A: call with theta + M_PI
    double s = sin(theta);
    double c = cos(theta);

    snapstack_msgs2::msg::Goal goal;
    goal.header.frame_id = "world";
    goal.p.x   = last_x + v*c*dt_;
    goal.p.y   = last_y + v*s*dt_;
    goal.p.z   = alt_;
    goal.v.x   = v*c;
    goal.v.y   = v*s;
    goal.v.z   = 0;
    goal.a.x = accel*c;
    goal.a.y = accel*s;
    goal.a.z = 0;
    goal.j.x  = 0;
    goal.j.y  = 0;
    goal.j.z  = 0;
    goal.psi = theta;
    goal.dpsi = 0;
    goal.power = true;

    return goal;
}

void Line::generateStopTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                            std::unordered_map<int,std::string>& index_msgs,
                            int& pub_index,
                            const rclcpp::Clock::SharedPtr& clock){
    // rclcpp::Time tstart = rclcpp::Time::now();
    rclcpp::Time tstart = clock->now();

    double v = sqrt(pow(goals[pub_index].v.x, 2) +
                    pow(goals[pub_index].v.y, 2));  // 2D current (goal) vel
    double theta = atan2(goals[pub_index].v.y,
                         goals[pub_index].v.x);  // current yaw

    std::vector<snapstack_msgs2::msg::Goal> goals_tmp;
    std::unordered_map<int,std::string> index_msgs_tmp;

    index_msgs_tmp[0] = "Line traj: pressed END, decelerating to 0 m/s";
    v = std::max(v - a3_*dt_, 0.0);
    goals_tmp.push_back(createLineGoal(goals[pub_index].p.x, goals[pub_index].p.y, v, -a3_, theta));

    while(v > 0){
        // generate points in the circle with *decreasing* velocity
        v = std::max(v - a3_*dt_, 0.0);
        goals_tmp.push_back(createLineGoal(goals_tmp.back().p.x, goals_tmp.back().p.y, v, -a3_, theta));
    }

    // we reached zero velocity
    index_msgs_tmp[goals_tmp.size() - 1] = "Line traj: stopped";

    goals = std::move(goals_tmp);
    index_msgs = std::move(index_msgs_tmp);
    pub_index = 0;

    // RCLCPP_INFO(logger_, "Time to calculate the braking traj (s): %f", (rclcpp::Time::now() - tstart).seconds());
    RCLCPP_INFO(logger_, "Time to calculate the braking traj (s): %f", (clock->now() - tstart).seconds());
    RCLCPP_INFO(logger_, "Goal vector size = %lu", goals.size());
}

bool Line::trajectoryInsideBounds(double xmin, double xmax,
                                  double ymin, double ymax,
                                  double zmin, double zmax){
    double d = (B_ - A_).norm();  // length of the segment
    double vg = v_goals_[0];  // TODO: multiple v_goals

    double d2 = get_d2();
    double t2 = d2 / vg;
    RCLCPP_INFO(logger_, "Line length: %f m", d);
    RCLCPP_INFO(logger_, "Constant velocity segment length: %f m", d2);
    RCLCPP_INFO(logger_, "Constant velocity segment time: %f s", t2);
    if(d2 < 0){
        RCLCPP_ERROR(logger_, "Line trajectory not feasible. Please increase accel, decrease v, or increase line length.");
        return false;
    }

    return
        isPointInsideBounds(xmin, xmax, ymin, ymax, zmin, zmax, A_) and
        isPointInsideBounds(xmin, xmax, ymin, ymax, zmin, zmax, B_);
}

double Line::get_d2() const{  // get length of constant velocity segment
    double d = (B_ - A_).norm();  // length of the full segment
    double vg = v_goals_[0];
    double d1 = 0.5*vg*vg/a1_;
    double d3 = 0.5*vg*vg/a3_;
    return d - d1 - d3;
}

} /* namespace */
