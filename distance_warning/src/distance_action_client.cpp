#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "distance_warning/action/check_distance.hpp"

using namespace std;
using namespace rclcpp;

class DistanceActionClient : public Node {
public:
    DistanceActionClient() : Node("distance_action_client") {
        this->client_ = rclcpp_action::create_client<distance_warning::action::CheckDistance>(this, "check_distance");
    }

    void send_goal(float dist) {
        if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }
        auto goal_msg = distance_warning::action::CheckDistance::Goal();
        goal_msg.distance_to_check = dist;
        RCLCPP_INFO(this->get_logger(), "Sending goal: check %.2f m", dist);

        auto options = rclcpp_action::Client<distance_warning::action::CheckDistance>::SendGoalOptions();

        options.goal_response_callback = bind(&DistanceActionClient::on_goal_response, this, placeholders::_1);
        options.feedback_callback = bind(&DistanceActionClient::on_feedback, this, placeholders::_1, placeholders::_2);
        options.result_callback = bind(&DistanceActionClient::on_result, this, placeholders::_1);

        this->client_->async_send_goal(goal_msg, options);
    }

private:
    rclcpp_action::Client<distance_warning::action::CheckDistance>::SharedPtr client_;

    void on_goal_response(
        rclcpp_action::ClientGoalHandle<distance_warning::action::CheckDistance>::SharedPtr goal_handle)
    {
        if (!goal_handle)
            RCLCPP_ERROR(this->get_logger(), "Goal refused!");
        else
            RCLCPP_INFO(this->get_logger(), "Goal accepted!");
    }

    void on_feedback(
        rclcpp_action::ClientGoalHandle<distance_warning::action::CheckDistance>::SharedPtr,
        const shared_ptr<const distance_warning::action::CheckDistance::Feedback> fb)
    {
        RCLCPP_INFO(this->get_logger(),
                    "[%d/%d]: %s",
                    fb->step,
                    fb->total_steps,
                    fb->feedback_msg.c_str());
    }

    void on_result(
        const rclcpp_action::ClientGoalHandle<distance_warning::action::CheckDistance>::WrappedResult & res)
    {
        if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
            if (res.result->is_safe)
                RCLCPP_INFO(this->get_logger(), "Result: SAFE");
            else
                RCLCPP_WARN(this->get_logger(), "Result: NOT SAFE");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Action failed!");
        }
    }
};

int main(int argc, char ** argv) {
    init(argc, argv);
    auto node = make_shared<DistanceActionClient>();
    // node->send_goal(0.3); // Gửi thử 0.3m
    spin(node);
    shutdown();
    return 0;
}