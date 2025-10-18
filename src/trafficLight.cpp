#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
using namespace std::chrono_literals;

class trafficLight : public rclcpp::Node {
    public:
        trafficLight() : Node("trafficLight"), state_("GREEN"), counter_(0) {
            this->declare_parameter<double>("redLightTime", 3.0);
            this->declare_parameter<double>("greenLightTime", 3.0);
            this->declare_parameter<double>("yellowLightTime", 2.0);
           
            pub_ = this->create_publisher<std_msgs::msg::String>("lightState", 10);
            timer_ = this->create_wall_timer(100ms, std::bind(&trafficLight::timer_callback, this));
        }
       
    private:
        void timer_callback() {
            counter_++;
           
            double redTime = this->get_parameter("redLightTime").as_double();
            double greenTime = this->get_parameter("greenLightTime").as_double();
            double yellowTime = this->get_parameter("yellowLightTime").as_double();
       
            int redTicks = redTime * 10;
            int greenTicks = greenTime * 10;
            int yellowTicks = yellowTime * 10;
       
            int totalTicks = greenTicks + yellowTicks + redTicks + yellowTicks;
            int currentTicks = counter_ % totalTicks;
           
            std::string newState;
            if (currentTicks < greenTicks) {
                newState = "GREEN";
            }
            else if (currentTicks < greenTicks + yellowTicks) {
                newState = "YELLOW";
            }
            else if (currentTicks < greenTicks + yellowTicks + redTicks) {
                newState = "RED";
            }
            else {
                newState = "YELLOW";
            }
           
            if (newState != state_) {
                state_ = newState;
                std_msgs::msg::String msg;
                msg.data = state_;
                pub_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Traffic light changed: %s", state_.c_str());
            }
        }
       
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::string state_;
        int counter_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<trafficLight>());
    rclcpp::shutdown();
    return 0;
}