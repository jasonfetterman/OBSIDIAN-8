#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <array>
#include <sstream>
#include <cmath>

using std::placeholders::_1;

class ReactiveCore : public rclcpp::Node
{
public:
    ReactiveCore()
    : Node("obsidian_core"),
      requested_motion_("IDLE"),
      current_state_(MotionState::IDLE),
      gait_phase_(0.0)
    {
        last_heartbeat_time_ = now();
        stand_entry_time_ = now();
        last_loop_time_ = now();

        leg_phase_offsets_ = {0.0, 0.5, 0.25, 0.75, 0.5, 0.0, 0.75, 0.25};

        heartbeat_sub_ = create_subscription<std_msgs::msg::String>(
            "/obsidian/heartbeat", 10,
            std::bind(&ReactiveCore::heartbeatCallback, this, _1));

        motion_sub_ = create_subscription<std_msgs::msg::String>(
            "/obsidian/motion_command", 10,
            std::bind(&ReactiveCore::motionCallback, this, _1));

        core_state_pub_ = create_publisher<std_msgs::msg::String>(
            "/obsidian/core_state", 10);

        motor_enable_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/obsidian/motor_command", 10);

        filtered_motion_pub_ = create_publisher<std_msgs::msg::String>(
            "/obsidian/filtered_motion_command", 10);

        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ReactiveCore::controlLoop, this));
    }

private:

    enum class MotionState
    {
        IDLE,
        STAND,
        WALK,
        SAFE_MODE
    };

    MotionState current_state_;

    rclcpp::Time last_heartbeat_time_;
    rclcpp::Time stand_entry_time_;
    rclcpp::Time last_loop_time_;

    std::string requested_motion_;

    double gait_phase_;
    std::array<double, 8> leg_phase_offsets_;

    void updateStateMachine()
    {
        if ((now() - last_heartbeat_time_).seconds() > 2.0)
        {
            current_state_ = MotionState::SAFE_MODE;
            return;
        }

        switch (current_state_)
        {
            case MotionState::SAFE_MODE:
                current_state_ = MotionState::IDLE;
                break;

            case MotionState::IDLE:
                if (requested_motion_ == "STAND")
                {
                    current_state_ = MotionState::STAND;
                    stand_entry_time_ = now();
                }
                break;

            case MotionState::STAND:
                if (requested_motion_ == "WALK")
                {
                    if ((now() - stand_entry_time_).seconds() > 1.0)
                    {
                        current_state_ = MotionState::WALK;
                        gait_phase_ = 0.0;
                        last_loop_time_ = now();
                    }
                }
                break;

            case MotionState::WALK:
                if (requested_motion_ == "STOP")
                {
                    current_state_ = MotionState::STAND;
                    stand_entry_time_ = now();
                }
                break;
        }
    }

    void updateGaitPhase()
    {
        if (current_state_ != MotionState::WALK)
            return;

        double dt = (now() - last_loop_time_).seconds();
        last_loop_time_ = now();

        double gait_frequency = 0.5;
        gait_phase_ += dt * gait_frequency;

        if (gait_phase_ >= 1.0)
            gait_phase_ -= 1.0;
    }

    void heartbeatCallback(const std_msgs::msg::String::SharedPtr)
    {
        last_heartbeat_time_ = now();
    }

    void motionCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        requested_motion_ = msg->data;
    }

    void controlLoop()
    {
        updateStateMachine();
        updateGaitPhase();
        publishOutputs();
    }

    void publishOutputs()
    {
        std_msgs::msg::String core_state_msg;
        std_msgs::msg::Bool motor_msg;
        std_msgs::msg::String filtered_motion_msg;

        std::stringstream output;

        switch (current_state_)
        {
            case MotionState::IDLE:
                core_state_msg.data = "IDLE";
                motor_msg.data = false;
                filtered_motion_msg.data = "STOP";
                break;

            case MotionState::STAND:
                core_state_msg.data = "STAND";
                motor_msg.data = true;
                filtered_motion_msg.data = "STAND";
                break;

            case MotionState::WALK:
                core_state_msg.data = "WALK";
                motor_msg.data = true;

                output << "FOOT_POS:";

                for (int i = 0; i < 8; i++)
                {
                    double leg_phase = gait_phase_ + leg_phase_offsets_[i];
                    if (leg_phase >= 1.0)
                        leg_phase -= 1.0;

                    double x, z;

                    if (leg_phase < 0.5)
                    {
                        // STANCE
                        double progress = leg_phase / 0.5;
                        x = 0.5 - progress;  // move backward
                        z = 0.0;
                    }
                    else
                    {
                        // SWING
                        double progress = (leg_phase - 0.5) / 0.5;
                        x = progress - 0.5;  // move forward
                        z = 0.2 * std::sin(progress * M_PI);  // arc lift
                    }

                    output << "(" << x << "," << z << ")";
                    if (i < 7)
                        output << ",";
                }

                filtered_motion_msg.data = output.str();
                break;

            case MotionState::SAFE_MODE:
                core_state_msg.data = "SAFE_MODE";
                motor_msg.data = false;
                filtered_motion_msg.data = "STOP";
                break;
        }

        core_state_pub_->publish(core_state_msg);
        motor_enable_pub_->publish(motor_msg);
        filtered_motion_pub_->publish(filtered_motion_msg);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr heartbeat_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_sub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr core_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_enable_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr filtered_motion_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveCore>());
    rclcpp::shutdown();
    return 0;
}
