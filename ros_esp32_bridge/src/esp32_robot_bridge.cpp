#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <string>
#include <vector>
#include <memory>
#include <chrono>

class Esp32RobotBridge : public rclcpp::Node {
private:
    // 串口相关
    boost::asio::io_service io_service_;
    boost::asio::serial_port* serial_port_;
    boost::thread serial_thread_;
    bool serial_running_;
    
    // ROS2发布器
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
    
    // 串口配置
    std::string port_name_;
    int baud_rate_;
    
    // 数据帧解析状态
    enum ParseState {
        WAITING_HEADER,
        WAITING_CMD,
        WAITING_DATA,
        WAITING_CHECKSUM,
        WAITING_TAIL
    };
    ParseState parse_state_;
    
    // 数据帧常量 - 适配ESP32发送的格式
    static const uint8_t FRAME_HEADER = 0xAA;
    static const uint8_t FRAME_TAIL = 0x55;
    static const uint8_t CMD_MOVE = 0x01;
    static const uint8_t CMD_DANCE = 0x02;
    static const uint8_t CMD_LIGHT = 0x03;
    static const uint8_t CMD_STOP = 0x04;
    
    // 数据帧缓冲区
    std::vector<uint8_t> frame_buffer_;
    
    // 速度控制参数
    double max_linear_speed_;
    double max_angular_speed_;
    double linear_scale_factor_;
    double angular_scale_factor_;
    
    // 目标位置控制
    struct TargetPosition {
        double target_x;           // 目标X位置 (米)
        double target_y;           // 目标Y位置 (米)
        double target_theta;       // 目标角度 (弧度)
        double current_x;          // 当前X位置 (米)
        double current_y;          // 当前Y位置 (米)
        double current_theta;      // 当前角度 (弧度)
        bool is_moving;            // 是否正在移动
        std::chrono::steady_clock::time_point start_time;  // 开始时间
        double duration;           // 目标持续时间 (秒)
    };
    TargetPosition target_pos_;
    
    // 位置到速度转换参数
    double position_tolerance_;    // 位置容差 (米)
    double angle_tolerance_;       // 角度容差 (弧度)
    double linear_p_gain_;        // 线速度P增益
    double angular_p_gain_;       // 角速度P增益
    double max_linear_accel_;     // 最大线加速度 (m/s²)
    double max_angular_accel_;    // 最大角加速度 (rad/s²)
    
    // 当前速度
    double current_linear_x_;
    double current_linear_y_;
    double current_angular_z_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;
    
public:
    Esp32RobotBridge() : 
        Node("esp32_robot_bridge"),
        serial_port_(nullptr),
        serial_running_(false),
        parse_state_(WAITING_HEADER),
        max_linear_speed_(1.0),
        max_angular_speed_(2.0),
        linear_scale_factor_(1.0),
        angular_scale_factor_(1.0),
        position_tolerance_(0.05),    // 5cm容差
        angle_tolerance_(0.1),        // 约5.7度容差
        linear_p_gain_(2.0),          // 线速度P增益
        angular_p_gain_(3.0),         // 角速度P增益
        max_linear_accel_(0.5),       // 0.5 m/s²
        max_angular_accel_(1.0),      // 1.0 rad/s²
        current_linear_x_(0.0),
        current_linear_y_(0.0),
        current_angular_z_(0.0) {
        
        // 初始化目标位置
        target_pos_.target_x = 0.0;
        target_pos_.target_y = 0.0;
        target_pos_.target_theta = 0.0;
        target_pos_.current_x = 0.0;
        target_pos_.current_y = 0.0;
        target_pos_.current_theta = 0.0;
        target_pos_.is_moving = false;
        target_pos_.duration = 0.0;
        
        // 初始化参数
        this->declare_parameter("port_name", "/dev/ttyUSB0");
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("max_linear_speed", 1.0);
        this->declare_parameter("max_angular_speed", 2.0);
        this->declare_parameter("linear_scale_factor", 1.0);
        this->declare_parameter("angular_scale_factor", 1.0);
        this->declare_parameter("position_tolerance", 0.05);
        this->declare_parameter("angle_tolerance", 0.1);
        this->declare_parameter("linear_p_gain", 2.0);
        this->declare_parameter("angular_p_gain", 3.0);
        this->declare_parameter("max_linear_accel", 0.5);
        this->declare_parameter("max_angular_accel", 1.0);
        
        port_name_ = this->get_parameter("port_name").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        linear_scale_factor_ = this->get_parameter("linear_scale_factor").as_double();
        angular_scale_factor_ = this->get_parameter("angular_scale_factor").as_double();
        position_tolerance_ = this->get_parameter("position_tolerance").as_double();
        angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
        linear_p_gain_ = this->get_parameter("linear_p_gain").as_double();
        angular_p_gain_ = this->get_parameter("angular_p_gain").as_double();
        max_linear_accel_ = this->get_parameter("max_linear_accel").as_double();
        max_angular_accel_ = this->get_parameter("max_angular_accel").as_double();
        
        // 初始化ROS2发布器
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        debug_pub_ = this->create_publisher<std_msgs::msg::String>("esp32_debug", 10);
        
        // 创建控制定时器 (50Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),  // 50Hz
            std::bind(&Esp32RobotBridge::controlLoop, this)
        );
        
        // 初始化串口
        if (initSerial()) {
            startSerialThread();
            RCLCPP_INFO(this->get_logger(), "ESP32 Robot Bridge initialized successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port");
        }
    }
    
    ~Esp32RobotBridge() {
        stopSerialThread();
        if (serial_port_) {
            serial_port_->close();
            delete serial_port_;
        }
    }
    
private:
    bool initSerial() {
        try {
            serial_port_ = new boost::asio::serial_port(io_service_, port_name_);
            serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
            serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
            serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
            
            RCLCPP_INFO(this->get_logger(), "Serial port opened: %s, baud rate: %d", port_name_.c_str(), baud_rate_);
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
            return false;
        }
    }
    
    void startSerialThread() {
        serial_running_ = true;
        serial_thread_ = boost::thread(&Esp32RobotBridge::serialReadLoop, this);
    }
    
    void stopSerialThread() {
        serial_running_ = false;
        if (serial_thread_.joinable()) {
            serial_thread_.join();
        }
    }
    
    void serialReadLoop() {
        uint8_t buffer[1];
        
        while (serial_running_) {
            try {
                boost::asio::read(*serial_port_, boost::asio::buffer(buffer, 1));
                processByte(buffer[0]);
            } catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Serial read error: %s", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
    
    void processByte(uint8_t byte) {
        switch (parse_state_) {
            case WAITING_HEADER:
                if (byte == FRAME_HEADER) {
                    frame_buffer_.clear();
                    frame_buffer_.push_back(byte);
                    parse_state_ = WAITING_CMD;
                }
                break;
                
            case WAITING_CMD:
                frame_buffer_.push_back(byte);
                parse_state_ = WAITING_DATA;
                break;
                
            case WAITING_DATA:
                frame_buffer_.push_back(byte);
                if (frame_buffer_.size() >= 7) {  // 8字节帧：AA + CMD + XH + XL + YH + YL + CHK + 55
                    parse_state_ = WAITING_CHECKSUM;
                }
                break;
                
            case WAITING_CHECKSUM:
                frame_buffer_.push_back(byte);
                parse_state_ = WAITING_TAIL;
                break;
                
            case WAITING_TAIL:
                frame_buffer_.push_back(byte);
                if (byte == FRAME_TAIL) {
                    processFrame();
                }
                parse_state_ = WAITING_HEADER;
                break;
        }
    }
    
    void processFrame() {
        if (frame_buffer_.size() < 8) {
            RCLCPP_WARN(this->get_logger(), "Incomplete frame received");
            return;
        }
        
        // 验证帧头
        if (frame_buffer_[0] != FRAME_HEADER) {
            RCLCPP_WARN(this->get_logger(), "Invalid frame header");
            return;
        }
        
        // 验证帧尾
        if (frame_buffer_[frame_buffer_.size() - 1] != FRAME_TAIL) {
            RCLCPP_WARN(this->get_logger(), "Invalid frame tail");
            return;
        }
        
        // 计算校验和（异或校验，不包括帧头）
        uint8_t calculated_checksum = 0;
        for (size_t i = 1; i < frame_buffer_.size() - 2; i++) {
            calculated_checksum ^= frame_buffer_[i];
        }
        
        if (calculated_checksum != frame_buffer_[frame_buffer_.size() - 2]) {
            RCLCPP_WARN(this->get_logger(), "Checksum error: calculated=0x%02X, received=0x%02X", 
                         calculated_checksum, frame_buffer_[frame_buffer_.size() - 2]);
            return;
        }
        
        // 解析命令
        uint8_t cmd = frame_buffer_[1];  // 命令在第二个字节
        switch (cmd) {
            case CMD_MOVE:
                processMoveCommand();
                break;
            case CMD_DANCE:
                processDanceCommand();
                break;
            case CMD_LIGHT:
                processLightCommand();
                break;
            case CMD_STOP:
                processStopCommand();
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown command: 0x%02X", cmd);
                break;
        }
        
        // 发布调试信息
        std::string debug_msg = "Frame: ";
        for (size_t i = 0; i < frame_buffer_.size(); i++) {
            char hex[8];
            snprintf(hex, sizeof(hex), "%02X ", frame_buffer_[i]);
            debug_msg += hex;
        }
        auto debug_pub_msg = std::make_unique<std_msgs::msg::String>();
        debug_pub_msg->data = debug_msg;
        debug_pub_->publish(*debug_pub_msg);
    }
    
    void processMoveCommand() {
        if (frame_buffer_.size() < 8) return;
        
        // 解析X和Y（16位整数，放大1000倍）
        // 协议：X=目标直线位移(米)，Y=目标转角(弧度)
        int16_t x_raw = (frame_buffer_[2] << 8) | frame_buffer_[3];  // 第3-4字节
        int16_t y_raw = (frame_buffer_[4] << 8) | frame_buffer_[5];  // 第5-6字节
        
        double distance_m = (double)x_raw / 1000.0;
        double angle_rad  = (double)y_raw / 1000.0;
        
        // 将位姿增量应用到目标
        target_pos_.target_x      = target_pos_.current_x + distance_m;
        target_pos_.target_y      = target_pos_.current_y; // 本方案不在平面Y方向平移
        target_pos_.target_theta  = target_pos_.current_theta + angle_rad;
        
        target_pos_.is_moving = true;
        target_pos_.start_time = std::chrono::steady_clock::now();
        target_pos_.duration = 0.0; // 非时间型目标
        
        RCLCPP_INFO(this->get_logger(), "Move target: dist=%.3f m, yaw=%.3f rad -> target_x=%.3f, target_theta=%.3f", 
                    distance_m, angle_rad, target_pos_.target_x, target_pos_.target_theta);
    }
    
    void convertSpeedToTargetPosition(double x_speed, double y_speed) {
        // 根据速度指令计算目标位置
        // 这里假设机器人以指定速度移动5秒到达目标位置
        
        double move_duration = 5.0;  // 默认移动5秒
        
        // 计算目标位置
        target_pos_.target_x = target_pos_.current_x + x_speed * move_duration;
        target_pos_.target_y = target_pos_.current_y + y_speed * move_duration;
        target_pos_.target_theta = target_pos_.current_theta;  // 保持当前角度
        
        // 设置移动状态
        target_pos_.is_moving = true;
        target_pos_.start_time = std::chrono::steady_clock::now();
        target_pos_.duration = move_duration;
        
        RCLCPP_INFO(this->get_logger(), "Converted speed to target: (%.3f, %.3f) -> (%.3f, %.3f)", 
                    x_speed, y_speed, target_pos_.target_x, target_pos_.target_y);
    }
    
    void processDanceCommand() {
        // 跳舞指令：原地旋转
        target_pos_.target_x = target_pos_.current_x;
        target_pos_.target_y = target_pos_.current_y;
        target_pos_.target_theta = target_pos_.current_theta + M_PI;  // 旋转180度
        
        target_pos_.is_moving = true;
        target_pos_.start_time = std::chrono::steady_clock::now();
        target_pos_.duration = 3.0;  // 跳舞3秒
        
        RCLCPP_INFO(this->get_logger(), "Dance command: rotate 180 degrees");
    }
    
    void processLightCommand() {
        if (frame_buffer_.size() < 8) return;
        
        uint8_t light_mode = frame_buffer_[5];  // 灯光模式在第6字节
        RCLCPP_INFO(this->get_logger(), "Light command: mode=%d", light_mode);
        // 灯光命令不影响移动，只记录日志
    }
    
    void processStopCommand() {
        // 停止指令：立即停止
        target_pos_.is_moving = false;
        current_linear_x_ = 0.0;
        current_linear_y_ = 0.0;
        current_angular_z_ = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "Stop command: immediate stop");
        publishTwist();
    }
    
    void controlLoop() {
        if (!target_pos_.is_moving) {
            return;
        }
        
        // 检查是否到达目标位置
        if (isTargetReached()) {
            target_pos_.is_moving = false;
            current_linear_x_ = 0.0;
            current_linear_y_ = 0.0;
            current_angular_z_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Target reached, stopping");
            publishTwist();
            return;
        }
        
        // 计算目标速度
        calculateTargetVelocity();
        
        // 应用速度限制和加速度限制
        applyVelocityLimits();
        
        // 更新当前位置
        updateCurrentPosition();
        
        // 发布速度命令
        publishTwist();
    }
    
    bool isTargetReached() {
        double dx = target_pos_.target_x - target_pos_.current_x;
        double dy = target_pos_.target_y - target_pos_.current_y;
        double dtheta = target_pos_.target_theta - target_pos_.current_theta;
        
        // 标准化角度差
        while (dtheta > M_PI) dtheta -= 2 * M_PI;
        while (dtheta < -M_PI) dtheta += 2 * M_PI;
        
        double position_error = sqrt(dx * dx + dy * dy);
        double angle_error = fabs(dtheta);
        
        return (position_error < position_tolerance_) && (angle_error < angle_tolerance_);
    }
    
    void calculateTargetVelocity() {
        // 计算位置误差
        double dx = target_pos_.target_x - target_pos_.current_x;
        double dy = target_pos_.target_y - target_pos_.current_y;
        double dtheta = target_pos_.target_theta - target_pos_.current_theta;
        
        // 标准化角度差
        while (dtheta > M_PI) dtheta -= 2 * M_PI;
        while (dtheta < -M_PI) dtheta += 2 * M_PI;
        
        // 计算目标速度（P控制器）
        double target_linear_x = linear_p_gain_ * dx;
        double target_linear_y = linear_p_gain_ * dy;
        double target_angular_z = angular_p_gain_ * dtheta;
        
        // 限制最大速度
        double linear_speed = sqrt(target_linear_x * target_linear_x + target_linear_y * target_linear_y);
        if (linear_speed > max_linear_speed_) {
            double scale = max_linear_speed_ / linear_speed;
            target_linear_x *= scale;
            target_linear_y *= scale;
        }
        
        if (fabs(target_angular_z) > max_angular_speed_) {
            target_angular_z = (target_angular_z > 0) ? max_angular_speed_ : -max_angular_speed_;
        }
        
        // 更新当前速度
        current_linear_x_ = target_linear_x;
        current_linear_y_ = target_linear_y;
        current_angular_z_ = target_angular_z;
    }
    
    void applyVelocityLimits() {
        // 应用加速度限制
        double dt = 0.02;  // 50Hz控制周期
        
        // 线速度加速度限制
        double linear_accel_x = (current_linear_x_ - current_linear_x_) / dt;
        double linear_accel_y = (current_linear_y_ - current_linear_y_) / dt;
        
        if (fabs(linear_accel_x) > max_linear_accel_) {
            linear_accel_x = (linear_accel_x > 0) ? max_linear_accel_ : -max_linear_accel_;
        }
        if (fabs(linear_accel_y) > max_linear_accel_) {
            linear_accel_y = (linear_accel_y > 0) ? max_linear_accel_ : -max_linear_accel_;
        }
        
        // 角速度加速度限制
        double angular_accel = (current_angular_z_ - current_angular_z_) / dt;
        if (fabs(angular_accel) > max_angular_accel_) {
            angular_accel = (angular_accel > 0) ? max_angular_accel_ : -max_angular_accel_;
        }
    }
    
    void updateCurrentPosition() {
        // 简单的积分更新（实际应用中应该使用里程计数据）
        double dt = 0.02;  // 50Hz控制周期
        
        // 更新位置
        target_pos_.current_x += current_linear_x_ * dt;
        target_pos_.current_y += current_linear_y_ * dt;
        target_pos_.current_theta += current_angular_z_ * dt;
        
        // 标准化角度
        while (target_pos_.current_theta > M_PI) target_pos_.current_theta -= 2 * M_PI;
        while (target_pos_.current_theta < -M_PI) target_pos_.current_theta += 2 * M_PI;
    }
    
    void publishTwist() {
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = current_linear_x_;
        twist_msg->linear.y = current_linear_y_;
        twist_msg->linear.z = 0.0;
        twist_msg->angular.x = 0.0;
        twist_msg->angular.y = 0.0;
        twist_msg->angular.z = current_angular_z_;
        
        cmd_vel_pub_->publish(*twist_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Published Twist: linear(%.3f, %.3f, 0.0), angular(0.0, 0.0, %.3f)", 
                     current_linear_x_, current_linear_y_, current_angular_z_);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<Esp32RobotBridge>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
} 