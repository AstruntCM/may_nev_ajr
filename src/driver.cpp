#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include <string>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

// színkódók => konzolhoz
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"

class turtleDriver : public rclcpp::Node {
    public:
        turtleDriver() : Node("turtleDriver"), lightState_("RED"), speed_(1.2), teleported_(false), turning_(false), currentY_(5.5), currentTheta_(0.0), targetTheta_(0.0) {
            teleportTurtle_ = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");

            setPen_ = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

            sub_ = this->create_subscription<std_msgs::msg::String>("lightState", 10, std::bind(&turtleDriver::light_callback, this, std::placeholders::_1));

            subPose_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&turtleDriver::pose_callback, this, std::placeholders::_1));

            pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
            
            RCLCPP_INFO(this->get_logger(), "Turtle driver node started");

            timer_ = this->create_wall_timer(1s, std::bind(&turtleDriver::teleportTurtle, this));

            timerM_ = this->create_wall_timer(100ms, std::bind(&turtleDriver::moveTurtle, this));
            
            // Toll letiltás
            disablePen();
        }
        
    private:
        void disablePen() {
            if (!setPen_->wait_for_service(2s)) {
                RCLCPP_WARN(this->get_logger(), "SetPen service not available!");
                return;
            }
            
            auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
            request->r = 0;
            request->g = 0;
            request->b = 0;
            request->width = 0;
            request->off = 1; 
            
            setPen_->async_send_request(request);
            RCLCPP_INFO(this->get_logger(), "Pen disabled - no line drawing");
        }

        void moveTurtle() {
            geometry_msgs::msg::Twist cmd;
            
            // Fordulás => megadott ponton
            if (turning_) {

                double angle_diff = targetTheta_ - currentTheta_;
                
                while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
                while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
                
                if (std::abs(angle_diff) > 0.087) {  // 0.087 rad ≈ 5 fok
                    cmd.angular.z = (angle_diff > 0) ? 3.14 : -3.14;
                    cmd.linear.x = 0.0;
                }
                // Fordulás befejezése
                else {
                    turning_ = false;
                    cmd.linear.x = 1.0;
                    cmd.angular.z = 0.0;
                    RCLCPP_INFO(this->get_logger(), "Turn completed");
                }
                pub_->publish(cmd);
                return;
            }
            
            // Fal detektálás + fordulás indítása
            if ((currentY_ > 9.5 || currentY_ < 1.5) && !turning_) {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                turning_ = true;
                targetTheta_ = currentTheta_ + M_PI; 
                
                while (targetTheta_ > M_PI) targetTheta_ -= 2 * M_PI;
                while (targetTheta_ < -M_PI) targetTheta_ += 2 * M_PI;
                
                RCLCPP_INFO(this->get_logger(), "Wall detected! Starting turn...");
                pub_->publish(cmd);
                return;
            }

            // Lámpa állapot alapján => sebesség álltítás
            if (lightState_ == "GREEN") {
                cmd.linear.x = speed_;
            }
            else if (lightState_ == "YELLOW") {
                cmd.linear.x = speed_ * 0.5;
            }
            else {  // RED
                cmd.linear.x = 0.0;
            }
            
            cmd.angular.z = 0.0;
            pub_->publish(cmd);
        }


        void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
            currentY_ = msg->y;
            currentTheta_ = msg->theta;
        }

        // Kezdo pozició => lentre teleportálás
        void teleportTurtle() {
            if (teleported_) return;

            if (!teleportTurtle_->wait_for_service(2s)) {
                RCLCPP_WARN(this->get_logger(), "Teleport service error!");
                return;
            }
            
            auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
            request->x = 5.5; 
            request->y = 2.0; 
            request->theta = 1.57;  // 90 fok (felfelé)
            
            teleportTurtle_->async_send_request(request);
            RCLCPP_INFO(this->get_logger(), "Turtle teleported to starting position");
            
            teleported_ = true;
            timer_->cancel();
        }

        void light_callback(const std_msgs::msg::String::SharedPtr msg) {
            lightState_ = msg->data;
            if (lightState_ == "RED") {
                RCLCPP_INFO(this->get_logger(), "%sLight is RED%s", RED, RESET);
            } 
            else if (lightState_ == "GREEN") {
                RCLCPP_INFO(this->get_logger(), "%sLight is GREEN%s", GREEN, RESET);
            } 
            else if (lightState_ == "YELLOW") {
                RCLCPP_INFO(this->get_logger(), "%sLight is YELLOW%s", YELLOW, RESET);
            }
        }
        
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subPose_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
        rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleportTurtle_;
        rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr setPen_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timerM_;
        std::string lightState_;
        double speed_;
        bool teleported_;
        bool turning_;
        double currentY_;
        double currentTheta_;
        double targetTheta_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<turtleDriver>());
    rclcpp::shutdown();
    return 0;
}