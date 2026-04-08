// //tên: set_threshold
// // kiểu dữ liệu: std_srvs/SetBool
// //Node: 
//   // nhận yêu cầu thay đổi giá trị threshold trong quá trình chạy
//     //request:
//       // nếu request có data:true thì giảm ngưỡng 0.1 (tối thiểu 0.1)
//       // Nếu request có data: false thì tăng ngưỡng 0.1 (tối đa 1.5)
//     //response:
//       //cập nhật giá trị mới lên parameter server
//       //trả về giá trị thành công và thông báo giá trị threshold mới 
// //luồng:  
//   // Node Listener đang chạy với ngưỡng 0.5
//   // Tôi dùng terminal gọi Service set_threshold với data:true
//   // Node Service nhận lệnh, tính threshold mới 0.5-0.1 =0.4
//   // cập nhật lên hệ thống 
//   //Node listener (sử dụng get_parameter mỗi lần có tin nhắn mới tới) -> cập nhật threshold ngay lập tức





#include "rclcpp/rclcpp.hpp"

#include "distance_warning/srv/set_threshold.hpp" //de ra sau khi chay terminal

using SetThreshold = distance_warning::srv::SetThreshold;
using namespace std;
using namespace rclcpp;
using placeholders::_1;
using placeholders::_2;

class SetThresholdService : public Node {
private:
    Service<SetThreshold>::SharedPtr service_;
    // Dùng Async để an toàn tuyệt đối, không gây treo Node
    shared_ptr<AsyncParametersClient>param_client_;
    
    

    void handleSetThreshold(const shared_ptr<SetThreshold::Request> request, 
                        shared_ptr<SetThreshold::Response> response) {
        
        double current = this->get_parameter("threshold").as_double();
        double new_threshold =current;
        if (request->increase == true) { 
            new_threshold += 0.1;
        } else { 
            new_threshold -= 0.1;
        }

        if (new_threshold < 0.1) new_threshold = 0.1;
        if (new_threshold > 1.5) new_threshold = 1.5;

        this->set_parameter({Parameter("threshold", new_threshold)});

        //async
        auto parameters = {rclcpp::Parameter("threshold", new_threshold)};
        param_client_->set_parameters(parameters);
        
        response->success = true;
        response ->new_threshold =new_threshold;
        response->message = "Threshold increased to: " + to_string(new_threshold) + " m";
        RCLCPP_INFO(this->get_logger(), "Threshold updated: %.2f -> %.2f",current,new_threshold);


        // ở trên chỉ update thr ở srv chứ không update thr ở listener -> make_shared
        
        // if (param_client_->wait_for_service(std::chrono::seconds(1))) {
        //     param_client_->set_parameters({
        //         rclcpp::Parameter("threshold", new_threshold)
        //     });
        //     RCLCPP_INFO(this->get_logger(), "Updated listener threshold: %.2f", new_threshold);
        // } else {
        //     RCLCPP_WARN(this->get_logger(), "Listener not available!");
        // }
    }

public:
    SetThresholdService() : Node("set_threshold_service") {

        this->declare_parameter("threshold", 0.5); 
        service_ = this->create_service<SetThreshold>(
            "set_threshold", bind(&SetThresholdService::handleSetThreshold, this, _1, _2));

        // param_client_ = std::make_shared<AsyncParametersClient>(this, "distance_listener"); 
        //khi thay đổi thr ở distance_listener thì ở đây tự động cập nhật, nhưng nếu listener chưa khởi tạo thì service sẽ không khởi tạo được do phụ thuộc listener
        // chưa kể, vì đây là hành động async (ko đồng bộ) -> có độ trễ (latency    )

        // để tránh sự phụ thuộc vào listener có thể khai báo thủ công bằng:
        //  this -> declare_parameter("threshold", 0.5);
        // make_shared
        param_client_ = std::make_shared<AsyncParametersClient>(this, "distance_listener"); // constructor của SyncParametersClient(NodeT * node, const std::string & name) -> cần con trỏ tới node và tên node -> (this, "tên_node")

        RCLCPP_INFO(this->get_logger(), "Service Set Threshold created!");
    }
};

int main(int argc, char **argv) {
    init(argc, argv);
    spin(make_shared<SetThresholdService>());
    shutdown();
    return 0;
}
//call service

// ros2 service call /set_threshold distance_warning/srv/SetThreshold "{increase: true}"
// ros2 service call /set_threshold distance_warning/srv/SetThreshold "{increase: false}"
// ros2 param set /distance_listener threshold 0.8