#include "rclcpp/rclcpp.hpp"
#include "snapstack_msgs2/msg/goal.hpp"
#include "snapstack_msgs2/msg/state.hpp"
#include "snapstack_msgs2/msg/quad_flight_mode.hpp"

#include <vector>
#include <string> 
#include <unordered_map>
#include <Eigen/Core>

namespace trajectory_generator { 

// class constructor
Doublet::Doublet(double alt, Eigen::Vector3d A,
         std::vector<double> v_goals, double period, double yaw, double decel, double dt): 
         alt_(alt), A_(A), v_goals_(v_goals), period_(period), theta_(yaw), decel_(decel), Trajectory(dt) {
}

Doublet::~Doublet(){
}

// genearate_traj file
void Doublet::generateTraj(std::vector<snapstack_msgs2::msg::Goal>& goals, 
                            std::unordered_map<int, std::string>& index_msgs, 
                            const rclcpp::Clock::SharedPtr& clock){

    /* 
    Description: Commands constant velocity doublet that will produce an (A_ - B_).norm() displacement from start position. 
    */

    // Get ros time 
    rclcpp::Time tstart = clock->now();

    // Append initial position to goals vector 
    double v = 0;
    goals.push_back(createLineGoal(A_.x(), A_.y(), v, 0, theta_)); // Initial position, velocity, acceleration

    //Compute t_AB
    double v_goal = v_goals_[0]; 
    // double d_AB = (B_ - A_).norm();
    // double t_AB = d_AB / v_goal;
    double t_AB = period_ / 2; 

    // Generate first step
    index_msgs[goals.size() - 1] = "Starting doublet"; 
    
    double t = 0;
    while(t < t_AB) { 
        goals.push_back(createLineGoal(goals.back().p.x, goals.back().p.y, v_goal, 0, theta_));
        t += dt_; 
    }

    // Generate step in opposite direction 
    index_msgs[goals.size() - 1] = "Reversing direction"; 
    t = 0;
    while (t < t_AB) {
        goals.push_back(createLineGoal(goals.back().p.x, goals.back().p.y, -v_goal, 0, theta_));
        t += dt_;
    }

    goals.push_back(createLineGoal(goals.back().p.x, goals.back().p.y, 0, 0, theta_)); 

    index_msgs[goals.size() - 1] = "Doublet complete"; 

    RCLCPP_INFO(logger_, "Time to complete trajectory (s): %f", (clock->now() - tstart).seconds());
    RCLCPP_INFO(logger_, "Goal vector size %lu", goals.size());
}

snapstack_msgs2::msg::Goal Doublet::createLineGoal(double last_x, double last_y, double v, double accel, double theta) const{

    double s = sin(theta);
    double c = cos(theta);

    // Populate goal
    snapstack_msgs2::msg::Goal goal;
    goal.header.frame_id = "world";
    // goal.p.x = last_x + v*c*dt_;
    // goal.p.y = last_y + v*s*dt_;
    goal.p.x = 0;
    goal.p.y = 0;
    goal.p.z = alt_;
    goal.v.x = v*c;
    goal.v.y = v*s;
    goal.v.z = 0;
    goal.a.x = 0;
    goal.a.y = 0;
    goal.a.z = 0;
    goal.j.x = 0;
    goal.j.y = 0;
    goal.j.z = 0;
    goal.psi = theta;
    goal.dpsi = 0;
    goal.power = true; 

    return goal; 
}

void Doublet::generateStopTraj(std::vector<snapstack_msgs2::msg::Goal>& goals, 
                               std::unordered_map<int, std::string>& index_msgs, 
                               int& pub_index, 
                               const rclcpp::Clock::SharedPtr& clock){
    //get ros time 
    rclcpp::Time tstart = clock->now();

    // Get magnitude of velocity and theta at stop time
    double v_stopped = sqrt(pow(goals[pub_index].v.x, 2) + pow(goals[pub_index].v.y, 2));
    double theta_stopped = atan2(goals[pub_index].v.y, goals[pub_index].v.x);

    // Make temporary vector for goals and dict for index_msgs
    std::vector<snapstack_msgs2::msg::Goal> goals_tmp;
    std::unordered_map<int, std::string> index_msgs_tmp;

    // Add initial element to goals vector 
    v_stopped = std::max(0.0, v_stopped - decel_*dt_);
    goals_tmp.push_back(createLineGoal(goals[pub_index].p.x, goals[pub_index].p.y, v_stopped, -decel_, theta_stopped));

    index_msgs_tmp[0] = "Decelerating to 0.0 m/s";

    // Decelerate to 0 m/s at -decel_ m/s^2
    while (v_stopped > 0){
        v_stopped = std::max(0.0, v_stopped - decel_*dt_);
        goals_tmp.push_back(createLineGoal(goals_tmp.back().p.x, goals_tmp.back().p.y, v_stopped, -decel_, theta_stopped));
    }

    index_msgs_tmp[goals_tmp.size() - 1] = "Vehicle stopped"; 

    goals = std::move(goals_tmp);
    index_msgs = std::move(index_msgs_tmp); 
    pub_index = 0;

    RCLCPP_INFO(logger_, "Time to decelerate (s): %f", (clock->now() - tstart).seconds());
    RCLCPP_INFO(logger_, "Size of goal vector: %lu", goals.size()); 
}

bool Doublet::trajectoryInsideBounds(double xmin, double xmax, 
                                    double ymin, double ymax, 
                                    double zmin, double zmax){
    
    // Use instant acceleration model to estimate out of bounds 
    Eigen::Vector3d B;
    B.x() = A_.x() + v_goals_[0] * period_ / 2;
    B.y() = A_.y() + v_goals_[0] * period_ / 2;
    B.z() = alt_;

    return
        isPointInsideBounds(xmin, xmax, ymin, ymax, zmin, zmax, A_) and
        isPointInsideBounds(xmin, xmax, ymin, ymax, zmin, zmax, B);
}

// TODO: Most likely can delete this 
double Doublet::get_d2() const{ 
    return 0;
}

} /*namespace*/
