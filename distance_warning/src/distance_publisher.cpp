// nhiệm vụ: quét, tính toán khoảng cách và truyền dữ liệu về khoảng cách
// Tên node: distance_publisher
// topic phát ra: distance_topic
// kiểu dữ liệu: std_msgs/Float32 (trong C++ ROS2 ghi là std_msgs::msg::Float32)
// logic: sinh ra 1 số thực (float) ngẫu nhiên từ 0.1 -> 1.5
// tần suất phát 1Hz -> mỗi 1s/1000ms sẽ đẩy data lên topic


using namespace std; //thư viện tiêu chuẩn
// dùng để viết ms, s thay vì tính toán 
#include<chrono>
using namespace chrono_literals; //để dùng hậu tố ms hay s cho thời gian
/////

#include<random> //thư viện sinh số ngẫu nhiên (ngon hơn rand())
#include<memory>

#include<rclcpp/rclcpp.hpp> // thư viện cốt lõi của ROS2 cho C++

#include "std_msgs/msg/float32.hpp" // thư viện định nghĩa kiểu msg Float32


using namespace rclcpp; //thư viện chuyên biệt cho ROS2

// kế thừa class Node để biến class này thành 1 node trong ROS2
class DistancePublisher: public Node{

private:
    // khai báo biến thành viên (member var) của class

    // SharedPtr giải quyết:
        // + Tiết kiệm bộ nhớ: ví dụ Node Pub gửi lên topic và có 10 Node Sub nhận dữ liệu đó, nó phải tạo ra 10 bản dữ liệu cho 10 Node Sub, thay vào đó dùng con trỏ "xài chung" thì nó chỉ tạo đúng 1 bản dữ liệu và gửi địa chỉ của bản đó đến 10 node kia 
        // + tự dọn rác: nếu mượn bộ nhớ mà quên trả, máy sẽ bị tràn ram. SharedPtr có bộ đếm:
            // khi Node cuối đọc xong và trả thẻ, nó sẽ tự động tiêu huỷ để giải phóng bộ nhớ
        // + Đảm bảo sẽ ko đi tiêu huỷ bản dữ liệu nếu còn node cần

    // Bind:
        // + mục tiêu là sau mỗi 1s thì gọi lại hàm timer_callback và chạy liên tục (until killed)
        // + bind(&DistancePublisher::timer_callback, this)):
            // &DistancePublisher::timer_callback có nghĩa là gọi hàm timer_callback tại địa chỉ của class DistancePublisher
            // this là con trỏ chỉ vào hàm DistancePublisher, do timer_callback nằm trong chính class này nên cần trỏ vào chính class này để chạy
    // 

    TimerBase::SharedPtr timer_;
    Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;

    //công cụ sinh số ngẫu nhiên
    default_random_engine generator_;
    uniform_real_distribution<float> distribution_;

    // hàm callback gọi lại sau mỗi 1 giây (in msg được push lên topic)
    void timer_callback(){
        // tạo biến msg giá trị Float32
        auto message = std_msgs::msg::Float32(); // msg type (cần đồng bộ với Sub để truyền được msg đi)

        // gán khoảng cách bằng số ngẫu nhiên sinh ra
        message.data = distribution_(generator_);

        // in ra màn hình log 
        RCLCPP_INFO(this->get_logger(), "Publishing: %.2f m", message.data);

        //phát tin nhắn lên topic
        publisher_->publish(message);
    }


public:
    // constructor: nơi khởi tạo Node và các thành phần bên trong
    DistancePublisher() : Node("distance_publisher"){
        //1. khởi tạo pub -> biến nó thành kênh truyền tin
        // phát dữ liệu lên topic 'distance_topic' với hàng đợi (về căn bản topic được mô phỏng bằng queue FIFO)
        publisher_ = this -> create_publisher<std_msgs::msg::Float32>("distance_topic", 10);// giả sử hàng đợi có size 10 (để lưu 10 tn gần nhất)


        //2. khởi tạo bộ sinh số ngẫu nhiên (0.1 - 1.5) 
        // Thiết lập seed để mỗi lần chạy tra một số khác nhau
        generator_.seed(random_device{}());
        distribution_ = uniform_real_distribution<float>(0.1f, 1.5f);

        //3. khởi tạo timer
        // f =1Hz tương đương 1000 ms (1s)
        // cứ mỗi 1s, hệ thống gọi lại hàm timer_callback()

        // biến timer_ này là biến được "lặp lại" (không chết sau khi constructor chết mà lặp lại sau 1s (do hàm create_wall_timer(thiết lập) và 'nguồn điện duy trì' spin))
        timer_ = this ->create_wall_timer( 
            1000ms, bind(&DistancePublisher::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Distance Publisher Node created");   
    }
    // gọi constructor xong là chết
};

int main(int argc, char* argv[]){
    // khoi tao moi truong ROS2
    init(argc, argv);

    // Tao node va nem no vao spin (loop vo han) de lien tuc chay
    spin(make_shared<DistancePublisher>()); // thay vì dùng new, dùng make_shared để tạo ra node được bảo vệ bởi shared_ptr (không xoá nhầm nếu hệ thống còn chạy)

    shutdown();
    return 0;
}


