#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <cmath>
using namespace std::chrono_literals;

class trafficLight : public rclcpp::Node {
    public:
        // Publisher Node létrehozása
        trafficLight() : Node("trafficLight"), state_("GREEN") {
            this->declare_parameter<double>("redLightTime", 3.0);
            this->declare_parameter<double>("greenLightTime", 3.0);

            pub_ = this->create_publisher<std_msgs::msg::String>("lightState", 10);
            // Időzitő létrehozzása 100ms
            timer_ = this->create_wall_timer(100ms, std::bind(&trafficLight::timer_callback, this));
            
            startTime_ = this->now();

            // Kezdeti lámpa állapot üzenet küldése
            std_msgs::msg::String msg;
            msg.data = state_;
            pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Traffic light initialized: %s", state_.c_str());
        }

    private:
        void timer_callback() {
            auto now = this->now(); // pillanatnyi idő
            double eLapsed = (now - startTime_).seconds(); // eltelt idő sec-ben
            double redTime = this->get_parameter("redLightTime").as_double();
            double greenTime = this->get_parameter("greenLightTime").as_double();
            double cycleT = redTime + greenTime; // ciklusidő sec-ben
            double phase = fmod(eLapsed, cycleT); // sec-ben hogy mennyi idő telt el a ciklusból.

            std::string newState = (phase < greenTime) ? "GREEN" : "RED" ;

            // Üzenet küldés a lámpa állapotáról => megváltozott
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
        rclcpp::Time startTime_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<trafficLight>());
    rclcpp::shutdown();
    return 0;
}