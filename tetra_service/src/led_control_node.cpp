#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include "interfaces/srv/led_control.hpp"
#include "interfaces/srv/led_toggle_control.hpp"
#include "interfaces/srv/toggle_on.hpp"

class LedControlNode : public rclcpp::Node
{
public:
    LedControlNode() : Node("led_control_node")
    {
		// ROS2 Service Client 생성
        led_control_client = this->create_client<interfaces::srv::LedControl>("led_cmd");
		led_toggle_control_client = this->create_client<interfaces::srv::LedToggleControl>("led_toggle_cmd");
		toggle_on_client = this->create_client<interfaces::srv::ToggleOn>("turn_on_cmd");

        // 메인 노드(tetra_service)에서 발행하는 로봇 상태 구독자 생성
        status_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/robot_status", 10,
            std::bind(&LedControlNode::statusCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "=======================================");
        RCLCPP_INFO(this->get_logger(), "    LED Control Node Initialized!!     ");
        RCLCPP_INFO(this->get_logger(), "=======================================");
    }

private:
    // 메인 상태 수신 콜백 (로봇 상태에 따른 LED 상태 분기)
    void statusCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int robot_status = msg->data;

        switch (robot_status) {
            case 0: // IDLE / Normal Drive (평상시 / 일반 주행)
                RCLCPP_INFO(this->get_logger(), "[LED] State: NORMAL / IDLE");
                LedToggleControl_Call(1, 3, 100, 3, 1);
                ToggleOn_Call(63);
                break;

            case 1: // DOCKING (충전 도킹 동작 중)
                RCLCPP_INFO(this->get_logger(), "[LED] State: DOCKING MOVEMENT");
                LedToggleControl_Call(1, 5, 100, 5, 1);
                ToggleOn_Call(30);
                break;

            case 99: // EMG PUSH (비상 정지 버튼 눌림)
                RCLCPP_WARN(this->get_logger(), "[LED] State: EMERGENCY STOP");
                LedToggleControl_Call(1, 10, 100, 10, 1);
                ToggleOn_Call(18);
                break;

            default:
                RCLCPP_INFO(this->get_logger(), "[LED] Unknown Robot State: %d", robot_status);
                break;
        }
    }

    // Client Call Functions
    void LedControl_Call(int id, int led_brightness) 
    {
        if (!led_control_client->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "led_cmd Service is not ready!");
            return;
        }

        auto request = std::make_shared<interfaces::srv::LedControl::Request>();
        request->id = id;
        request->led_brightness = led_brightness;

        auto result_future = led_control_client->async_send_request(
            request, std::bind(&LedControlNode::LedControl_callback, this, std::placeholders::_1));
    }

    void LedToggleControl_Call(int de_index, int light_accel, int led_high_brightness, int light_decel, int led_low_brightness) 
    {
        if (!led_toggle_control_client->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "led_toggle_cmd Service is not ready!");
            return;
        }

        auto request = std::make_shared<interfaces::srv::LedToggleControl::Request>();
        request->de_index = de_index;
        request->light_accel = light_accel;
        request->led_high_brightness = led_high_brightness;
        request->light_decel = light_decel;
        request->led_low_brightness = led_low_brightness;

        auto result_future = led_toggle_control_client->async_send_request(
            request, std::bind(&LedControlNode::LedToggleControl_callback, this, std::placeholders::_1));
    }

    void ToggleOn_Call(int id) 
    {
        if (!toggle_on_client->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "turn_on_cmd Service is not ready!");
            return;
        }

        auto request = std::make_shared<interfaces::srv::ToggleOn::Request>();
        request->id = id;

        auto result_future = toggle_on_client->async_send_request(
            request, std::bind(&LedControlNode::ToggleOn_callback, this, std::placeholders::_1));
    }

    // Client Callback Functions
    void LedControl_callback(rclcpp::Client<interfaces::srv::LedControl>::SharedFuture future)
    {
        (void)future; // Unused warning 방지
        // RCLCPP_INFO(this->get_logger(), "response LedControl_callback");
    }

    void LedToggleControl_callback(rclcpp::Client<interfaces::srv::LedToggleControl>::SharedFuture future)
    {
        (void)future;
        // RCLCPP_INFO(this->get_logger(), "response LedToggleControl_callback");
    }

    void ToggleOn_callback(rclcpp::Client<interfaces::srv::ToggleOn>::SharedFuture future)
    {
        (void)future;
        // RCLCPP_INFO(this->get_logger(), "response ToggleOn_callback");
    }

    // 멤버 변수
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr status_sub_;

    rclcpp::Client<interfaces::srv::LedControl>::SharedPtr led_control_client;
    rclcpp::Client<interfaces::srv::LedToggleControl>::SharedPtr led_toggle_control_client;
    rclcpp::Client<interfaces::srv::ToggleOn>::SharedPtr toggle_on_client;
};

// Main Function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}