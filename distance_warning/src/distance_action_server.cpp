#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "distance_warning/action/check_distance.hpp"
#include <thread>

using namespace std;
using namespace rclcpp;

class DistanceActionServer : public Node {
public:
    DistanceActionServer() : Node("distance_action_server") {
        // Khởi tạo Action Server
        this->action_server_ = rclcpp_action::create_server<distance_warning::action::CheckDistance>(
            this, "check_distance",
            bind(&DistanceActionServer::handle_goal, this, placeholders::_1, placeholders::_2),
            bind(&DistanceActionServer::handle_cancel, this, placeholders::_1),
            bind(&DistanceActionServer::handle_accepted, this, placeholders::_1));
        
        // Để lấy threshold từ Parameter Server (của Node Listener)
        param_client_ = std::make_shared<AsyncParametersClient>(this, "distance_listener");
    }

private:
    rclcpp_action::Server<distance_warning::action::CheckDistance>::SharedPtr action_server_;
    shared_ptr<AsyncParametersClient> param_client_;

    // Chấp nhận Goal
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, shared_ptr<const distance_warning::action::CheckDistance::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Chấp nhận Hủy
    rclcpp_action::CancelResponse handle_cancel(const shared_ptr<rclcpp_action::ServerGoalHandle<distance_warning::action::CheckDistance>>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Bắt đầu thực hiện
    void handle_accepted(const shared_ptr<rclcpp_action::ServerGoalHandle<distance_warning::action::CheckDistance>> goal_handle) {
        thread{bind(&DistanceActionServer::execute, this, placeholders::_1), goal_handle}.detach();
    }

    void execute(const shared_ptr<rclcpp_action::ServerGoalHandle<distance_warning::action::CheckDistance>> goal_handle) {
        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<distance_warning::action::CheckDistance::Feedback>();
        auto result = std::make_shared<distance_warning::action::CheckDistance::Result>();

        // 1. Giả lập 5 bước delay với feedback
        vector<string>steps = {"Receiving distance value...",
        "Fetching threshold parameter...",
        "Comparing values...",
        "Generating result...",
        "Done."
        };
        int total = steps.size();
        for (int i = 0; i < 5; i++) {
            feedback->step = i + 1;
            feedback->feedback_msg = steps[i];
            feedback->total_steps = total;

            goal_handle->publish_feedback(feedback);
            this_thread::sleep_for(chrono::milliseconds(500));
        }

        // 2. Lấy threshold 
        double threshold = 0.5; // Mặc định
        auto future = param_client_->get_parameters({"threshold"});
        if (future.wait_for(chrono::seconds(1)) == future_status::ready) {
            threshold = future.get()[0].as_double();
        }

        // 3. Kiểm tra và trả kết quả
        result->is_safe = (goal->distance_to_check >= threshold);
        result->result_message = result->is_safe ? "Safe distance!" : "Warning: Too close to obstacle!";
        goal_handle->succeed(result);
    }
};

int main(int argc, char ** argv) {
    init(argc, argv);
    spin(make_shared<DistanceActionServer>());
    shutdown();
    return 0;
}

// goi action de kiem tra 
// ros2 action send_goal --feedback /distance_action distance_warning/action/Distance "{target_distance: 1.5}"
//ros2 action send_goal /check_distance distance_warning/action/CheckDistance "{distance_to_check: 0.8}"