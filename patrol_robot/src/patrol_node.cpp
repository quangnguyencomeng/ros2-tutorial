#include <chrono>
#include <memory>
#include <tuple>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
using namespace std;
using namespace chrono_literals;
using namespace placeholders;


class PatrolNode : public rclcpp::Node
{
public:
  PatrolNode() : Node("patrol_node"), m_current_index(0){
    m_nav_client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  
    //4 waypoints on map (rviz2), use comm:  ros2 topic echo /clicked_point
    m_waypoints ={
        {1.5228875875473022,-1.5027711391448975,-0.001434326171875},
        {1.6150468587875366,1.595870852470398,-0.005340576171875},
        {-1.6925221681594849,1.5239757299423218,0.002471923828125},
        {-1.8264621496200562,-1.3134897947311401,0.002471923828125}
    };

    SendNextGoal();
  }      

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  

  rclcpp_action::Client<NavigateToPose>::SharedPtr m_nav_client;

  // Danh sách waypoint: {x, y, yaw}
  std::vector<std::tuple<double, double, double>> m_waypoints;
  size_t m_current_index{0};
  
  rclcpp::TimerBase::SharedPtr m_timer;

  void SendNextGoal(){
    if (!m_nav_client -> wait_for_action_server(5s)){
        RCLCPP_ERROR(get_logger(), "Nav action server not ready!");
        return;
    }
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp    = this->get_clock()->now();

    //present coords
    double x, y,yaw;
    tie(x,y,yaw) = m_waypoints[m_current_index]; //tie = tuple

    goal_msg.pose.pose.position.x  = x;
    goal_msg.pose.pose.position.y  = y;

    // Chuyển đổi góc Yaw (radian) sang Quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal_msg.pose.pose.orientation.x = q.x();
    goal_msg.pose.pose.orientation.y = q.y();
    goal_msg.pose.pose.orientation.z = q.z();
    goal_msg.pose.pose.orientation.w = q.w();


    //start to move
    RCLCPP_INFO(this->get_logger(), "[Start] Heading to waypoint %zu: (x = %.2f, y = %.2f, z = %.2f)", m_current_index, x,y,yaw);
    
    //callback 
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&PatrolNode::ResultCallback, this, _1);
    

    m_nav_client -> async_send_goal(goal_msg, send_goal_options);
  }
  void ResultCallback(const GoalHandleNav::WrappedResult & result){
    switch (result.code){
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this ->get_logger(), "[SUCCEEDED] Moved to waypoint %zu", m_current_index);
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this-> get_logger(), "[ABORTED] Goal aborted at waypoint %zu, moving to next waypoint", m_current_index);
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "[CANCELLED] Goal is cancelled.");
            return; 
        default:
            RCLCPP_ERROR(this->get_logger(), "[ERROR] Unknown.");
            break;

    }
    m_current_index = (m_current_index+1)%m_waypoints.size();
    m_timer = this ->create_wall_timer( 
            3s, bind(&PatrolNode::SendNextGoal, this));
    

  }
};

void graceful_shutdown(){
    RCLCPP_INFO(rclcpp::get_logger("patrol_node"), "Control C -> Shutting down...");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  // Đăng ký hàm xử lý shutdown
  rclcpp::on_shutdown(graceful_shutdown);

  auto node = std::make_shared<PatrolNode>();

  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}