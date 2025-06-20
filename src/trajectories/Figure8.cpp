/**
 * @file Figure8.cpp
 * @brief Figure8 class
 * @author Jialin Chen
 * @date 2025-06-18
 */

#include "trajectory_generator_ros2/trajectories/Figure8.hpp"

#include "snapstack_msgs2/msg/quad_flight_mode.hpp"
#include "snapstack_msgs2/msg/state.hpp"
#include "snapstack_msgs2/msg/goal.hpp"

#include <vector>
#include <unordered_map>
#include <string>
#include <Eigen/Core>

namespace trajectory_generator {

Figure8::Figure8(double alt, double r, double cx, double cy,
           std::vector<double> v_goals, double t_traj, double accel, double dt):
      alt_(alt), r_(r), cx_(cx), cy_(cy),
      v_goals_(v_goals), t_traj_(t_traj), accel_(accel), Trajectory(dt){}
      
Figure8::~Figure8()
{
}

void Figure8::generateTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                          std::unordered_map<int,std::string>& index_msgs,
                          const rclcpp::Clock::SharedPtr& clock)
{
    // rclcpp::Time tstart = rclcpp::Time::now();
    rclcpp::Time tstart = clock->now();

    // init pos
    double theta = 0;
    double v = 0;

    goals.push_back(createFigure8Goal(v, 0, theta));

    for(int i = 0; i < v_goals_.size(); ++i){
        double v_goal = v_goals_[i];
        index_msgs[goals.size() - 1] = "Figure8 traj: accelerating to " + std::to_string(v_goal) + " m/s";
        // accelerate to the goal velocity
        while(v < v_goal){
            // generate points in the Figure8 with *increasing* velocity
            v = std::min(v + accel_*dt_, v_goal);
            double omega = v/r_;
            theta += omega*dt_;

            goals.push_back(createFigure8Goal(v, accel_, theta));
        }

        // we reached v_goal, continue the traj for t_traj_ seconds
        if(fabs(v - v_goal) > 0.001){
            RCLCPP_WARN(logger_, "Vels are not in increasing order, ignoring vels from the first to decrease...");
        }

        index_msgs[goals.size() - 1] = "Figure8 traj: reached " + std::to_string(v_goal)
                + " m/s, keeping constant v for " + std::to_string(t_traj_) + " s";
        double current_t_traj_ = 0;
        while(current_t_traj_ < t_traj_){
            // generate points in the Figure8 with *constant* velocity v == v_goal
            double omega = v/r_;
            theta += omega*dt_;

            goals.push_back(createFigure8Goal(v, 0, theta));
            current_t_traj_ += dt_;
        }
    }
    // now decelerate to 0
    index_msgs[goals.size() - 1] = "Figure8 traj: decelerating to 0 m/s";
    while(v > 0){
        // generate points in the figure8 with *decreasing* velocity
        v = std::max(v - accel_*dt_, 0.0);
        double omega = v/r_;
        theta += omega*dt_;

        goals.push_back(createFigure8Goal(v, -accel_, theta));
    }

    // we reached zero velocity
    if(fabs(v) > 0.001){
        RCLCPP_ERROR(logger_, "Error: final velocity is not zero");
        exit(1);
    }
    index_msgs[goals.size() - 1] = "Figure 8 traj: stopped";

    // RCLCPP_INFO(logger_, "Time to calculate the traj (s): %f", (rclcpp::Time::now() - tstart).seconds());
    RCLCPP_INFO(logger_, "Time to calculate the traj (s): %f", (clock->now() - tstart).seconds());
    RCLCPP_INFO(logger_, "Goal vector size = %lu", goals.size());
}

snapstack_msgs2::msg::Goal Figure8::createFigure8Goal(double v, double accel, double theta) const{
    // TODO: return ref to goal to avoid copy? Same for the simpleInterpolation function
    double s = sin(theta);
    double c = cos(theta);
    double sc = s * c;
    // double v2r  = pow(v,2)/r_;
    // double v3r2 = pow(v,3)/pow(r_,2);
    // double v4r3 = pow(v,4)/pow(r_,3);
    double omega = v/r_;

    snapstack_msgs2::msg::Goal goal;
    goal.header.frame_id = "world";
    goal.p.x   = cx_ + r_*s;
    goal.p.y   = cy_ + r_*s*c;
    goal.p.z   = alt_;
    goal.v.x   = r_ * omega * c;
    goal.v.y   = r_ * omega * (c * c - s * s);
    goal.v.z   = 0;
    goal.a.x = -r_ * omega * omega * s; //+ accel*c; //this accel term makes a.x discontinuous
    goal.a.y = -4 * r_ * omega * omega * sc; //+ accel*s; //this accel term makes a.y discontinuous
    goal.a.z = 0;
    goal.j.x  = 0;
    goal.j.y  = 0;
    goal.j.z  = 0;
    //goal.s.x  = v4r3*c;
    //goal.s.y  = v4r3*s;
    //goal.s.z  = 0;
    goal.psi = atan2(goal.v.y, goal.v.x); // face direction of velocity
    goal.dpsi = omega;  // dyaw has to be in world frame, the outer loop already converts it to body
    goal.power = true;

    return goal;
}

void Figure8::generateStopTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                              std::unordered_map<int,std::string>& index_msgs,
                              int& pub_index,
                              const rclcpp::Clock::SharedPtr& clock){

    // rclcpp::Time tstart = rclcpp::Time::now();
    rclcpp::Time tstart = clock->now();

    double v = sqrt(pow(goals[pub_index].v.x, 2) +
                    pow(goals[pub_index].v.y, 2));  // 2D current (goal) vel
    double theta = atan2(goals[pub_index].p.y - cy_,
                         goals[pub_index].p.x - cx_);  // current (goal) angle wrt the center

    std::vector<snapstack_msgs2::msg::Goal> goals_tmp;
    std::unordered_map<int,std::string> index_msgs_tmp;

    index_msgs_tmp[0] = "Figure8 traj: pressed END, decelerating to 0 m/s";

    while(v > 0){
        // generate points in the Figure8 with *decreasing* velocity
        v = std::max(v - accel_*dt_, 0.0);
        double omega = v/r_;
        theta += omega*dt_;

        goals_tmp.push_back(createFigure8Goal(v, -accel_, theta));
    }

    // we reached zero velocity
    index_msgs_tmp[goals_tmp.size() - 1] = "Figure8 traj: stopped";

    goals = std::move(goals_tmp);
    index_msgs = std::move(index_msgs_tmp);
    pub_index = 0;

    // RCLCPP_INFO(logger_, "Time to calculate the braking traj (s): %f", (rclcpp::Time::now() - tstart).seconds());
    RCLCPP_INFO(logger_, "Time to calculate the braking traj (s): %f", (clock->now() - tstart).seconds());
    RCLCPP_INFO(logger_, "Goal vector size = %lu", goals.size());
}

bool Figure8::trajectoryInsideBounds(double xmin, double xmax,
                                    double ymin, double ymax,
                                    double zmin, double zmax){
  return
      isPointInsideBounds(xmin, xmax, ymin, ymax, zmin, zmax,
                          Eigen::Vector3d(cx_ - r_, cy_ - r_, alt_)) and
      isPointInsideBounds(xmin, xmax, ymin, ymax, zmin, zmax,
                          Eigen::Vector3d(cx_ + r_, cy_ + r_, alt_));
}

} /* namespace */
