#ifndef COMMAND_TO_MAVROS_H
#define COMMAND_TO_MAVROS_H

#include <bitset>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <math_utils.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

using namespace std;

namespace namespace_command_to_mavros {

    class command_to_mavros {
    public:
        /**
         * @brief 构造函数初始化命令发送到MAVROS节点的参数和订阅者。
         *
         * 构造函数中主要进行以下操作：
         * 1. 初始化ROS节点句柄并设置一些参数，如起飞高度和地理围栏边界。
         * 2. 初始化与无人机状态和位置相关的变量。
         * 3. 订阅无人机的状态、位置、速度、姿态等信息。
         * 4. 设置发布器用于发送无人机的目标位置和姿态等命令。
         * 5. 初始化用于控制无人机解锁和切换飞行模式的服务客户端。
         *
         * 这些操作为后续的无人机控制和状态监测奠定了基础。
         */
        command_to_mavros(void) : command_nh("~") {
            // 从ROS参数服务器获取起飞高度参数，默认值为1.0米
            command_nh.param<float>("Takeoff_height", Takeoff_height, 1.0);

            // 从ROS参数服务器获取地理围栏的边界参数，设定默认值为-100.0到100.0米
            command_nh.param("geo_fence/x_min", geo_fence_x[0], -100.0);
            command_nh.param("geo_fence/x_max", geo_fence_x[1], 100.0);
            command_nh.param("geo_fence/y_min", geo_fence_y[0], -100.0);
            command_nh.param("geo_fence/y_max", geo_fence_y[1], 100.0);
            command_nh.param("geo_fence/z_min", geo_fence_z[0], -100.0);
            command_nh.param("geo_fence/z_max", geo_fence_z[1], 100.0);

            // 初始化无人机在飞控坐标系下的当前位置、速度、姿态等变量
            pos_drone_fcu = Eigen::Vector3d(0.0, 0.0, 0.0);
            vel_drone_fcu = Eigen::Vector3d(0.0, 0.0, 0.0);
            q_fcu         = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
            Euler_fcu     = Eigen::Vector3d(0.0, 0.0, 0.0);
            rates_fcu     = Eigen::Vector3d(0.0, 0.0, 0.0);

            // 初始化无人机的目标位置、速度、加速度等变量
            pos_drone_fcu_target   = Eigen::Vector3d(0.0, 0.0, 0.0);
            vel_drone_fcu_target   = Eigen::Vector3d(0.0, 0.0, 0.0);
            accel_drone_fcu_target = Eigen::Vector3d(0.0, 0.0, 0.0);
            q_fcu_target           = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
            Euler_fcu_target       = Eigen::Vector3d(0.0, 0.0, 0.0);
            Thrust_target          = 0.0;

            // 初始化起飞和着陆位置以及保持位置和航向
            Takeoff_position = Eigen::Vector3d(0.0, 0.0, 0.0);
            Hold_position    = Eigen::Vector3d(0.0, 0.0, 0.0);
            Hold_yaw         = 0.0;
            type_mask_target = 0;
            frame_target     = 0;

            Land_position          = Eigen::Vector3d(0.0, 0.0, 0.0);
            Land_yaw               = 0.0;
            flag_set_land_position = 0;

            // 订阅无人机状态、位置、速度和姿态信息
            state_sub = command_nh.subscribe<mavros_msgs::State>(
                "/mavros/state", 10, &command_to_mavros::state_cb, this);

            position_sub = command_nh.subscribe<geometry_msgs::PoseStamped>(
                "/mavros/local_position/pose", 100, &command_to_mavros::pos_cb,
                this);

            velocity_sub = command_nh.subscribe<geometry_msgs::TwistStamped>(
                "/mavros/local_position/velocity_local", 100,
                &command_to_mavros::vel_cb, this);

            attitude_sub = command_nh.subscribe<sensor_msgs::Imu>(
                "/mavros/imu/data", 10, &command_to_mavros::att_cb, this);

            // 订阅无人机的目标姿态和位置
            attitude_target_sub =
                command_nh.subscribe<mavros_msgs::AttitudeTarget>(
                    "/mavros/setpoint_raw/target_attitude", 10,
                    &command_to_mavros::att_target_cb, this);

            position_target_sub =
                command_nh.subscribe<mavros_msgs::PositionTarget>(
                    "/mavros/setpoint_raw/target_local", 10,
                    &command_to_mavros::pos_target_cb, this);

            actuator_target_sub =
                command_nh.subscribe<mavros_msgs::ActuatorControl>(
                    "/mavros/target_actuator_control", 10,
                    &command_to_mavros::actuator_target_cb, this);

            // 设置发布器用于发送无人机的本地导航目标和作动器控制命令
            setpoint_raw_local_pub =
                command_nh.advertise<mavros_msgs::PositionTarget>(
                    "/mavros/setpoint_raw/local", 10);

            actuator_setpoint_pub =
                command_nh.advertise<mavros_msgs::ActuatorControl>(
                    "/mavros/actuator_control", 10);

            // 初始化用于解锁和切换飞行模式的服务客户端
            arming_client = command_nh.serviceClient<mavros_msgs::CommandBool>(
                "/mavros/cmd/arming");

            set_mode_client = command_nh.serviceClient<mavros_msgs::SetMode>(
                "/mavros/set_mode");
        }

        // Geigraphical fence
        Eigen::Vector2d geo_fence_x;
        Eigen::Vector2d geo_fence_y;
        Eigen::Vector2d geo_fence_z;

        // Takeoff Height
        float Takeoff_height;

        // Takeoff Position of the Drone
        Eigen::Vector3d Takeoff_position;

        // Hold Position of the Drone (For hold mode in command.msg)
        Eigen::Vector3d Hold_position;
        double Hold_yaw;

        // Land Position of the Drone
        Eigen::Vector3d Land_position;
        double Land_yaw;
        int flag_set_land_position;

        // Current state of the drone
        mavros_msgs::State current_state;

        // Current pos of the drone
        Eigen::Vector3d pos_drone_fcu;
        // Current vel of the drone
        Eigen::Vector3d vel_drone_fcu;

        // Current att of the drone
        Eigen::Quaterniond q_fcu;
        Eigen::Vector3d Euler_fcu;
        Eigen::Vector3d rates_fcu;

        // Target typemask [from fcu]
        int type_mask_target;

        int frame_target;

        // Target pos of the drone [from fcu]
        Eigen::Vector3d pos_drone_fcu_target;

        // Target vel of the drone [from fcu]
        Eigen::Vector3d vel_drone_fcu_target;

        // Target accel of the drone [from fcu]
        Eigen::Vector3d accel_drone_fcu_target;

        // Target att of the drone [from fcu]
        Eigen::Quaterniond q_fcu_target;
        Eigen::Vector3d Euler_fcu_target;

        // Target thrust of the drone [from fcu]
        float Thrust_target;

        mavros_msgs::ActuatorControl actuator_target;

        mavros_msgs::ActuatorControl actuator_setpoint;

        mavros_msgs::SetMode mode_cmd;

        mavros_msgs::CommandBool arm_cmd;

        ros::ServiceClient arming_client;

        ros::ServiceClient set_mode_client;

        void set_takeoff_position() {
            Takeoff_position = pos_drone_fcu;
        }

        // Takeoff to the default altitude, pls change the param in PX4:
        // MIS_TAKEOFF_ALT
        void takeoff();

        // Land to current position
        void land();

        // Idle. Do nothing.
        void idle();

        // Loiter in the current position.
        void loiter();

        // Send pos_setpoint and yaw_setpoint in ENU frame to PX4
        void send_pos_setpoint(Eigen::Vector3d pos_sp, float yaw_sp);

        // Send vel_setpoint and yaw_setpoint in ENU frame to PX4
        void send_vel_setpoint(Eigen::Vector3d vel_sp, float yaw_sp);

        // Send pos_setpoint and yaw_setpoint in body frame to PX4
        void send_vel_setpoint_body(Eigen::Vector3d vel_sp, float yaw_sp);

        // Send accel_setpoint and yaw_setpoint in ENU frame to PX4
        void send_accel_setpoint(Eigen::Vector3d accel_sp, float yaw_sp);

        // Send actuator_setpoint to PX4[Not recommanded. Because the high delay
        // between the onboard computer and Pixhawk]
        void send_actuator_setpoint(Eigen::Vector4d actuator_sp);

        // Printf the parameters
        void printf_param();

        // Pringt the drone state[Full state]
        void prinft_drone_state(float current_time);

        // Pringt the drone state[Simple state]
        void prinft_drone_state2(float current_time);

        // Check for failsafe
        void check_failsafe();

        // printf the geo fence
        void show_geo_fence();

    private:
        ros::NodeHandle command_nh;
        ros::Subscriber state_sub;
        ros::Subscriber position_sub;
        ros::Subscriber velocity_sub;
        ros::Subscriber attitude_sub;
        ros::Subscriber attitude_target_sub;
        ros::Subscriber position_target_sub;
        ros::Subscriber actuator_target_sub;
        ros::Publisher setpoint_raw_local_pub;
        ros::Publisher actuator_setpoint_pub;

        void state_cb(const mavros_msgs::State::ConstPtr& msg) {
            current_state = *msg;
        }

        void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
            pos_drone_fcu =
                Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y,
                                msg->pose.position.z);
        }

        void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
            vel_drone_fcu = Eigen::Vector3d(
                msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
        }

        void att_cb(const sensor_msgs::Imu::ConstPtr& msg) {
            q_fcu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x,
                                       msg->orientation.y, msg->orientation.z);

            // Transform the Quaternion to Euler Angles
            Euler_fcu = quaternion_to_euler(q_fcu);

            rates_fcu = Eigen::Vector3d(msg->angular_velocity.x,
                                        msg->angular_velocity.y,
                                        msg->angular_velocity.z);
        }

        void att_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg) {
            q_fcu_target =
                Eigen::Quaterniond(msg->orientation.w, msg->orientation.x,
                                   msg->orientation.y, msg->orientation.z);

            // Transform the Quaternion to Euler Angles
            Euler_fcu_target = quaternion_to_euler(q_fcu_target);

            Thrust_target = msg->thrust;
        }

        void pos_target_cb(const mavros_msgs::PositionTarget::ConstPtr& msg) {
            type_mask_target = msg->type_mask;

            frame_target = msg->coordinate_frame;

            pos_drone_fcu_target = Eigen::Vector3d(
                msg->position.x, msg->position.y, msg->position.z);

            vel_drone_fcu_target = Eigen::Vector3d(
                msg->velocity.x, msg->velocity.y, msg->velocity.z);

            accel_drone_fcu_target = Eigen::Vector3d(
                msg->acceleration_or_force.x, msg->acceleration_or_force.y,
                msg->acceleration_or_force.z);
        }

        void
        actuator_target_cb(const mavros_msgs::ActuatorControl::ConstPtr& msg) {
            actuator_target = *msg;
        }
    };

    void command_to_mavros::takeoff() {
        mavros_msgs::PositionTarget pos_setpoint;

        pos_setpoint.type_mask = 0x1000;

        setpoint_raw_local_pub.publish(pos_setpoint);
    }

    /**
     * @brief 实现无人机着陆功能
     *
     * 本函数通过设置无人机的位置目标来实现着陆过程。如果着陆位置未被设定，
     * 则使用当前无人机位置作为着陆点，并记录当前姿态作为着陆时的yaw角度。
     * 使用mavros_msgs::PositionTarget消息来设定无人机的位置目标，包括xyz位置和yaw角度。
     * 如果无人机高度接近起飞高度，会切换无人机模式为手动，并解除武装。
     */
    void command_to_mavros::land() {
        // 如果着陆位置未设置，则使用当前无人机位置作为着陆点，并从起飞位置获取z轴高度
        if (flag_set_land_position == 0) {
            Land_position[0]       = pos_drone_fcu[0];
            Land_position[1]       = pos_drone_fcu[1];
            Land_position[2]       = Takeoff_position[2];
            Land_yaw               = Euler_fcu[2];
            flag_set_land_position = 1;
        }

        // 初始化位置目标消息，设置类型掩码和坐标框架
        mavros_msgs::PositionTarget pos_setpoint;
        // 设置类型掩码，允许控制xyz位置和yaw角度
        pos_setpoint.type_mask = 0b100111111000;   // 100 111 111 000  xyz + yaw
        pos_setpoint.coordinate_frame = 1;         // 使用局部坐标框架

        // 设置位置目标为着陆位置
        pos_setpoint.position.x = Land_position[0];
        pos_setpoint.position.y = Land_position[1];
        pos_setpoint.position.z = Land_position[2];
        // 设置yaw角度为着陆时的姿态
        pos_setpoint.yaw = Land_yaw;

        //如果距离起飞高度小于20厘米，则直接上锁并切换为手动模式；
        if (abs(pos_drone_fcu[2] - Takeoff_position[2]) < (0.2)) {
            // 如果当前模式为OFFBOARD，则切换为MANUAL模式
            if (current_state.mode == "OFFBOARD") {
                mode_cmd.request.custom_mode = "MANUAL";
                set_mode_client.call(mode_cmd);
            }
            // 如果无人机已武装，则解除武装
            if (current_state.armed) {
                arm_cmd.request.value = false;
                arming_client.call(arm_cmd);
            }
            // 如果解除武装成功，给出反馈
            if (arm_cmd.response.success) {
                cout << "Disarm successfully!" << endl;
            }
        } else {
            // 如果高度足够，发布位置目标消息，控制无人机着陆
            setpoint_raw_local_pub.publish(pos_setpoint);
            cout << "The drone is landing " << endl;
        }
    }

    void command_to_mavros::loiter() {
        mavros_msgs::PositionTarget pos_setpoint;

        // Here pls ref to mavlink_receiver.cpp
        pos_setpoint.type_mask = 0x3000;

        setpoint_raw_local_pub.publish(pos_setpoint);
    }

        /**
     * @brief 使飞行器进入空闲状态
     * 
     * 该函数通过发送一个位置目标消息来指示飞行器进入空闲状态。
     * 它设置了一个特定的类型掩码，该掩码指示飞行器应该停止任何移动，
     * 并保持当前位置。这是通过发布一个定位目标消息到 MAVROS 的主题来实现的。
     * 
     * @note 设置类型掩码为0x4000表示位置控制模式下不进行任何移动。
     */
    void command_to_mavros::idle() {
        // 创建一个定位目标消息，用于设置飞行器的位置目标
        mavros_msgs::PositionTarget pos_setpoint;

        // 设置类型掩码，0x4000表示没有移动指令，即保持当前位置
        pos_setpoint.type_mask = 0x4000;

        // 发布定位目标消息，使飞行器进入空闲状态
        setpoint_raw_local_pub.publish(pos_setpoint);
    }

    void command_to_mavros::send_pos_setpoint(Eigen::Vector3d pos_sp,
                                              float yaw_sp) {
        mavros_msgs::PositionTarget pos_setpoint;
        // Bitmask toindicate which dimensions should be ignored (1 means
        // ignore,0 means not ignore; Bit 10 must set to 0) Bit 1:x, bit 2:y,
        // bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az,
        // bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate Bit 10 should set to
        // 0, means is not force sp
        pos_setpoint.type_mask = 0b100111111000;   // 100 111 111 000  xyz + yaw

        // uint8 FRAME_LOCAL_NED = 1
        // uint8 FRAME_BODY_NED = 8
        pos_setpoint.coordinate_frame = 1;

        pos_setpoint.position.x = pos_sp[0];
        pos_setpoint.position.y = pos_sp[1];
        pos_setpoint.position.z = pos_sp[2];

        pos_setpoint.yaw = yaw_sp * M_PI / 180;

        setpoint_raw_local_pub.publish(pos_setpoint);
    }

    void command_to_mavros::send_vel_setpoint(Eigen::Vector3d vel_sp,
                                              float yaw_sp) {
        mavros_msgs::PositionTarget pos_setpoint;

        pos_setpoint.type_mask = 0b100111000111;

        pos_setpoint.coordinate_frame = 1;

        pos_setpoint.velocity.x = vel_sp[0];
        pos_setpoint.velocity.y = vel_sp[1];
        pos_setpoint.velocity.z = vel_sp[2];

        pos_setpoint.yaw = yaw_sp * M_PI / 180;

        setpoint_raw_local_pub.publish(pos_setpoint);
    }

    void command_to_mavros::send_vel_setpoint_body(Eigen::Vector3d vel_sp,
                                                   float yaw_sp) {
        mavros_msgs::PositionTarget pos_setpoint;

        pos_setpoint.type_mask = 0b100111000111;

        // uint8 FRAME_LOCAL_NED = 1
        // uint8 FRAME_BODY_NED = 8
        pos_setpoint.coordinate_frame = 8;

        pos_setpoint.position.x = vel_sp[0];
        pos_setpoint.position.y = vel_sp[1];
        pos_setpoint.position.z = vel_sp[2];

        pos_setpoint.yaw = yaw_sp * M_PI / 180;

        setpoint_raw_local_pub.publish(pos_setpoint);
    }

    /**
     * @brief 发送加速度设定点到MAVROS
     *
     * 该函数用于构造并发布一个加速度设定点的消息，该消息会被MAVROS接收并用于控制无人机的运动。
     * 设定点包括三个轴的加速度和一个偏航角度。
     *
     * @param accel_sp
     * 加速度设定点，一个三维向量，表示在X、Y、Z轴上的加速度要求。
     * @param yaw_sp 偏航设定点，以度为单位，表示无人机的偏航方向。
     */
    void command_to_mavros::send_accel_setpoint(Eigen::Vector3d accel_sp,
                                                float yaw_sp) {
        // 初始化位置设定点消息
        mavros_msgs::PositionTarget pos_setpoint;

        // 设置类型掩码，定义了消息中哪些字段是有效的
        // 此处的掩码表示使用本地NED坐标系，且设定点包括加速度信息
        pos_setpoint.type_mask = 0b100000111111;

        // 设置坐标系，1表示本地坐标系
        pos_setpoint.coordinate_frame = 1;

        // 设置三个轴的加速度设定点
        pos_setpoint.acceleration_or_force.x = accel_sp[0];
        pos_setpoint.acceleration_or_force.y = accel_sp[1];
        pos_setpoint.acceleration_or_force.z = accel_sp[2];

        // 将偏航角度转换为弧度，并设置到设定点中
        pos_setpoint.yaw = yaw_sp * M_PI / 180;

        // 发布加速度设定点消息
        setpoint_raw_local_pub.publish(pos_setpoint);
    }

    void
    command_to_mavros::send_actuator_setpoint(Eigen::Vector4d actuator_sp) {
        actuator_setpoint.group_mix   = 0;
        actuator_setpoint.controls[0] = actuator_sp(0);
        actuator_setpoint.controls[1] = actuator_sp(1);
        actuator_setpoint.controls[2] = actuator_sp(2);
        actuator_setpoint.controls[3] = actuator_sp(3);
        actuator_setpoint.controls[4] = 0.0;
        actuator_setpoint.controls[5] = 0.0;
        actuator_setpoint.controls[6] = 0.0;
        actuator_setpoint.controls[7] = 0.0;

        actuator_setpoint_pub.publish(actuator_setpoint);
    }

    /**
     * @brief 展示地理围栏信息
     *
     * 该函数用于在控制台上打印地理围栏的边界坐标。地理围栏是一个设定的区域，
     * 用于限制无人机的活动范围。通过打印x、y、z三个方向的最小和最大坐标值，
     * 可以让操作员了解无人机被允许飞行的区域范围。
     *
     * @note
     * 该函数假定geo_fence_x、geo_fence_y和geo_fence_z为全局变量或类成员变量，
     *       并且它们是包含两个元素的数组，分别表示最小值和最大值。
     */
    void command_to_mavros::show_geo_fence() {
        // 打印地理围栏的x坐标范围
        cout << "geo_fence_x : " << geo_fence_x[0] << " [m]  to  "
             << geo_fence_x[1] << " [m]" << endl;
        // 打印地理围栏的y坐标范围
        cout << "geo_fence_y : " << geo_fence_y[0] << " [m]  to  "
             << geo_fence_y[1] << " [m]" << endl;
        // 打印地理围栏的z坐标范围
        cout << "geo_fence_z : " << geo_fence_z[0] << " [m]  to  "
             << geo_fence_z[1] << " [m]" << endl;
    }

    /**
     * @brief 检查无人机是否超出地理围栏范围
     *
     * 本函数定期检查无人机的当前位置是否位于地理围栏内。
     * 如果无人机位置超出任一围栏边界，则触发紧急降落程序，
     * 直到无人机成功降落或ROS节点关闭。
     *
     * @note 依赖于ros::ok()来检查ROS节点是否仍在运行。
     */
    void command_to_mavros::check_failsafe() {
        // 检查无人机在X、Y、Z三个维度是否超出地理围栏范围
        if (pos_drone_fcu[0] < geo_fence_x[0] ||
            pos_drone_fcu[0] > geo_fence_x[1] ||
            pos_drone_fcu[1] < geo_fence_y[0] ||
            pos_drone_fcu[1] > geo_fence_y[1] ||
            pos_drone_fcu[2] < geo_fence_z[0] ||
            pos_drone_fcu[2] > geo_fence_z[1]) {
            // 当无人机位置超出地理围栏时，循环尝试降落无人机
            while (ros::ok()) {
                land();
                cout << "Out of the geo fence, the drone is landing... "
                     << endl;
            }
        }
    }

    // 【打印参数函数】
    void command_to_mavros::printf_param() {
        cout << ">>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<" << endl;

        cout << "takeoff_height : " << Takeoff_height << endl;
    }

    void command_to_mavros::prinft_drone_state(float current_time) {
        cout << ">>>>>>>>>>>>>>>Drone State<<<<<<<<<<<<<<<<<<" << endl;
        //固定的浮点显示
        cout.setf(ios::fixed);
        // setprecision(n) 设显示小数精度为n位
        cout << setprecision(2);
        //左对齐
        cout.setf(ios::left);
        // 强制显示小数点
        cout.setf(ios::showpoint);
        // 强制显示符号
        cout.setf(ios::showpos);

        cout << setprecision(1);

        cout << "Time: " << current_time << " [s] ";

        //是否和飞控建立起连接
        if (current_state.connected == true) {
            cout << " [ Connected ]  ";
        } else {
            cout << " [ Unconnected ]  ";
        }

        //是否上锁
        if (current_state.armed == true) {
            cout << "  [ Armed ]   ";
        } else {
            cout << "  [ DisArmed ]   ";
        }

        cout << " [ " << current_state.mode << " ]   " << endl;

        cout << setprecision(2);

        cout << "Position [X Y Z] : " << pos_drone_fcu[0] << " [ m ] "
             << pos_drone_fcu[1] << " [ m ] " << pos_drone_fcu[2] << " [ m ] "
             << endl;
        cout << "Velocity [X Y Z] : " << vel_drone_fcu[0] << " [m/s] "
             << vel_drone_fcu[1] << " [m/s] " << vel_drone_fcu[2] << " [m/s] "
             << endl;

        cout << "Attitude [R P Y] : " << Euler_fcu[0] * 180 / M_PI << " [deg] "
             << Euler_fcu[1] * 180 / M_PI << " [deg] "
             << Euler_fcu[2] * 180 / M_PI << " [deg] " << endl;

        cout << "Acc_target [X Y Z] : " << accel_drone_fcu_target[0]
             << " [m/s^2] " << accel_drone_fcu_target[1] << " [m/s^2] "
             << accel_drone_fcu_target[2] << " [m/s^2] " << endl;

        cout << "Att_target [R P Y] : " << Euler_fcu_target[0] * 180 / M_PI
             << " [deg] " << Euler_fcu_target[1] * 180 / M_PI << " [deg] "
             << Euler_fcu_target[2] * 180 / M_PI << " [deg] " << endl;

        cout << "Thr_target [0 - 1] : " << Thrust_target << endl;

        // ned to enu
        cout << "actuator_target [0 1 2 3] : " << actuator_target.controls[0]
             << " [ ] " << -actuator_target.controls[1] << " [ ] "
             << -actuator_target.controls[2] << " [ ] "
             << actuator_target.controls[3] << " [ ] " << endl;

        cout << "actuator_target [4 5 6 7] : " << actuator_target.controls[4]
             << " [ ] " << actuator_target.controls[5] << " [ ] "
             << actuator_target.controls[6] << " [ ] "
             << actuator_target.controls[7] << " [ ] " << endl;

        // cout << "Thank you for your support!                    ----Amov Lab"
        // <<endl;
    }

    /**
     * @brief 打印无人机状态信息
     *
     * 此函数用于格式化并打印当前无人机的状态，包括连接状态、武装状态、模式、位置、速度和姿态信息。
     * 通过控制台输出，便于操作员了解无人机的实时状态。
     *
     * @param current_time 当前时间，以秒为单位
     */
    void command_to_mavros::prinft_drone_state2(float current_time) {
        // 打印标题
        cout << ">>>>>>>>>>Drone State<<<<<<<<<<<<<<" << endl;

        // 设置输出格式：固定小数点，左对齐，显示小数点，显示正号
        // 固定的浮点显示
        cout.setf(ios::fixed);
        // setprecision(n) 设显示小数精度为n位
        cout << setprecision(2);
        //左对齐
        cout.setf(ios::left);
        // 强制显示小数点
        cout.setf(ios::showpoint);
        // 强制显示符号
        cout.setf(ios::showpos);

        // 打印当前时间
        cout << "Time: " << fixed << setprecision(1) << current_time << " [s] ";

        // 根据连接状态打印连接标识
        //是否和飞控建立起连接
        if (current_state.connected == true) {
            cout << " [ Connected ]  ";
        } else {
            cout << " [ Unconnected ]  ";
        }

        // 根据武装状态打印武装标识
        //是否上锁
        if (current_state.armed == true) {
            cout << "  [ Armed ]   ";
        } else {
            cout << "  [ DisArmed ]   ";
        }

        // 打印当前模式
        cout << " [ " << current_state.mode << " ]   " << endl;

        // 打印无人机的位置信息
        cout << "Position [X Y Z] : " << fixed << setprecision(2)
             << pos_drone_fcu[0] << " [ m ] " << pos_drone_fcu[1] << " [ m ] "
             << pos_drone_fcu[2] << " [ m ] " << endl;
        // 打印无人机的速度信息
        cout << "Velocity [X Y Z] : " << vel_drone_fcu[0] << " [m/s] "
             << vel_drone_fcu[1] << " [m/s] " << vel_drone_fcu[2] << " [m/s] "
             << endl;
        // 打印无人机的姿态信息（角度）
        cout << "Attitude [R P Y] : " << Euler_fcu[0] * 180 / M_PI << " [deg] "
             << Euler_fcu[1] * 180 / M_PI << " [deg] "
             << Euler_fcu[2] * 180 / M_PI << " [deg] " << endl;
        // 打印无人机的目标姿态信息（角度）
        cout << "Att_target [R P Y] : " << Euler_fcu_target[0] * 180 / M_PI
             << " [deg] " << Euler_fcu_target[1] * 180 / M_PI << " [deg] "
             << Euler_fcu_target[2] * 180 / M_PI << " [deg] " << endl;
    }

}   // namespace namespace_command_to_mavros
#endif
