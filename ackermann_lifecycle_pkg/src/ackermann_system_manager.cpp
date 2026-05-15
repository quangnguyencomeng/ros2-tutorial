#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class AckermannSystemManager : public rclcpp::Node {
public:
    AckermannSystemManager() : Node("ackermann_system_manager") {
        // --- ĐIỂM QUAN TRỌNG NHẤT ---
        // Tạo một node ẩn riêng biệt để Client không bị đụng độ Executor với Node chính
        client_node_ = rclcpp::Node::make_shared("manager_client_node_hidden");
        client_ = client_node_->create_client<lifecycle_msgs::srv::ChangeState>("/ackermann_kinematics_node/change_state");

        start_srv_ = create_service<std_srvs::srv::Trigger>(
            "/system/start", std::bind(&AckermannSystemManager::handle_start, this, std::placeholders::_1, std::placeholders::_2));
        stop_srv_ = create_service<std_srvs::srv::Trigger>(
            "/system/stop", std::bind(&AckermannSystemManager::handle_stop, this, std::placeholders::_1, std::placeholders::_2));

        status_pub_ = create_publisher<std_msgs::msg::String>("/system/status", 10);
        
        RCLCPP_INFO(get_logger(), "Manager ready.");
    }

private:
    bool call_transition(uint8_t transition_id) {
        if (!client_->wait_for_service(2s)) return false;
        auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        req->transition.id = transition_id;
        
        auto result = client_->async_send_request(req);
        
        // Spin con node ẨN thay vì node CHÍNH
        if (rclcpp::spin_until_future_complete(client_node_, result) != rclcpp::FutureReturnCode::SUCCESS) return false;
        return result.get()->success;
    }

    void handle_start(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        if (call_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
            if (call_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
                res->success = true;
                res->message = "System is ACTIVE";
                publish_status("ACTIVE");
                return;
            }
            call_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
        }
        res->success = false;
        publish_status("ERROR_START_FAILED");
    }

    void handle_stop(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        call_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        call_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
        res->success = true;
        publish_status("INACTIVE");
    }

    void publish_status(std::string status) {
        auto msg = std_msgs::msg::String();
        msg.data = status;
        status_pub_->publish(msg);
    }

    rclcpp::Node::SharedPtr client_node_; // Khai báo Node ẩn
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_, stop_srv_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannSystemManager>());
    rclcpp::shutdown();
    return 0;
}