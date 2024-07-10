#include <command_to_mavros.h>
#include <pos_controller_PID.h>
#include <px4_command/command.h>
#include <ros/ros.h>

//*************************//dyxtest
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
//************************//

#include <Eigen/Eigen>

using namespace std;

using namespace namespace_command_to_mavros;
using namespace namespace_PID;

//自定义的Command变量
//相应的命令分别为
//移动(惯性系ENU)，移动(机体系)，悬停，降落，上锁，紧急降落，待机
enum Command {
    Move_ENU,
    Move_Body,
    Hold,
    Takeoff,
    Land,
    Arm,
    Disarm,
    Failsafe_land,
    Idle
};

Eigen::Vector3d pos_sp(0, 0, 0);
Eigen::Vector3d vel_sp(0, 0, 0);
double yaw_sp = 0;
Eigen::Vector3d accel_sp(0, 0, 0);

px4_command::command Command_Now;    //无人机当前执行命令
px4_command::command Command_Last;   //无人机上一条执行命令

float get_ros_time(ros::Time begin);
void prinft_command_state();
void rotation_yaw(float yaw_angle, float input[2], float output[2]);
void Command_cb(const px4_command::command::ConstPtr& msg) {
    Command_Now = *msg;
}

int main(int argc, char** argv) {
    /// 初始化ROS节点，命名为"px4_pos_controller"
    ros::init(argc, argv, "px4_pos_controller");

    /// 创建一个节点句柄，用于后续的ROS通信
    ros::NodeHandle nh("~");

    /// 订阅名为"/px4/command"的主题，接收px4_command::command类型的消息
    /// 通过回调函数Command_cb处理接收到的消息
    ros::Subscriber Command_sub =
        nh.subscribe<px4_command::command>("/px4/command", 10, Command_cb);

    /// 创建一个发布者，发布名为"/mavros/setpoint_position/local"的主题
    /// 发布geometry_msgs::PoseStamped类型的消息
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/mavros/setpoint_position/local", 10);

    /// 设置循环频率为50Hz
    ros::Rate rate(50.0);

    /// 初始化位置控制器对象
    command_to_mavros pos_controller;

    /// 调用函数打印参数信息
    pos_controller.printf_param();

    /// 显示地理围栏信息
    pos_controller.show_geo_fence();

    /// 初始化位置控制器的PID控制器对象
    pos_controller_PID pos_controller_pid;

    /// 调用函数打印PID参数信息
    pos_controller_pid.printf_param();

    /// 当ROS节点正常运行且位置控制器与飞控连接时，进入循环
    /// 等待和飞控的连接
    while (ros::ok() && pos_controller.current_state.connected) {
        /// 处理一次ROS消息
        ros::spinOnce();
        /// 按照设定的频率休眠
        rate.sleep();
        /// 当与飞控未连接时，打印提示信息
        ROS_INFO("Not Connected");
    }

    /// 当与飞控连接成功时，打印提示信息
    // 连接成功
    ROS_INFO("Connected!!");

    // 初始化一个geometry_msgs::PoseStamped类型的变量pose，用于存储位姿信息
    geometry_msgs::PoseStamped pose;
    // 将位姿的初始位置设置在坐标原点
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    // 初始化一个循环变量i，用于控制循环次数
    // 先读取一些飞控的数据
    int i = 0;
    // 循环50次，发布初始位姿到本地位置发布者
    for (i = 0; i < 50; i++) {
        // 发布位姿
        local_pos_pub.publish(pose);
        // 处理ROS的其他消息
        ros::spinOnce();
        // 等待一定时间，控制发布频率
        rate.sleep();
    }

    // 调用pos_controller的set_takeoff_position函数，设置起飞位置
    pos_controller.set_takeoff_position();

    // 初始化Command_Now变量，用于发送控制命令
    // 初始化命令-
    // 默认设置：Idle模式 电机怠速旋转 等待来自上层的控制指令
    Command_Now.comid   = 0;
    Command_Now.command = Idle;

    // 记录当前时间作为开始时间，用于计算时间差
    // 记录启控时间
    ros::Time begin_time = ros::Time::now();
    // 获取当前ROS时间并存储为last_time，用于后续计算时间差
    float last_time = get_ros_time(begin_time);
    // 初始化时间差变量dt
    float dt = 0;
    while (ros::ok()) {
        // 执行一次ROS消息循环，处理新到来的消息。
        // 执行回调函数
        ros::spinOnce();

        // 获取当前ROS时间，并计算自上次更新以来的时间差。
        float cur_time = get_ros_time(begin_time);
        dt             = cur_time - last_time;

        // 限制时间差的范围在0.01到0.03秒之间。
        // 这是为了确保控制系统的时间响应在可接受的范围内。
        dt = constrain_function2(dt, 0.01, 0.03);

        // 更新上一次更新时间。
        last_time = cur_time;

        // 检查飞行器是否超出地理围栏。
        // 如果超出，将执行紧急降落。
        pos_controller.check_failsafe();

        // 打印飞行器当前状态。
        // Printf the drone state
        pos_controller.prinft_drone_state2(cur_time);

        // 打印当前命令状态。
        // Printf the command state
        prinft_command_state();

        // 如果上一个命令是着陆命令，则当前命令也应为着陆命令。
        // 这是为了确保在任何情况下，只要收到着陆命令，飞行器就会执行着陆操作。
        // 无人机一旦接受到Land指令，则会屏蔽其他指令
        if (Command_Last.command == Land) {
            Command_Now.command = Land;
        }

        switch (Command_Now.command) {
        case Move_ENU:
            // 根据当前指令设置位置、速度目标点
            pos_sp =
                Eigen::Vector3d(Command_Now.pos_sp[0], Command_Now.pos_sp[1],
                                Command_Now.pos_sp[2]);
            vel_sp =
                Eigen::Vector3d(Command_Now.vel_sp[0], Command_Now.vel_sp[1],
                                Command_Now.vel_sp[2]);
            // 根据当前状态和目标点计算加速度指令
            accel_sp = pos_controller_pid.pos_controller(
                pos_controller.pos_drone_fcu, pos_controller.vel_drone_fcu,
                pos_sp, vel_sp, Command_Now.sub_mode, dt);
            // 发送加速度指令和yaw目标角度
            pos_controller.send_accel_setpoint(accel_sp, Command_Now.yaw_sp);
            break;

        case Move_Body:
            // 只处理命令ID增加的情况，避免重复处理旧命令
            // 只有在comid增加时才会进入解算
            if (Command_Now.comid > Command_Last.comid) {
                // 根据飞行器当前yaw角度，将body
                // frame下的速度或位置目标转换为ENU frame xy velocity mode
                if (Command_Now.sub_mode & 0b10) {
                    float d_vel_body[2] = {
                        Command_Now.vel_sp[0],
                        Command_Now.vel_sp[1]};   // the desired xy velocity in
                                                  // Body Frame
                    float d_vel_enu[2];   // the desired xy velocity in NED
                                          // Frame

                    rotation_yaw(pos_controller.Euler_fcu[2], d_vel_body,
                                 d_vel_enu);
                    vel_sp[0] = d_vel_enu[0];
                    vel_sp[1] = d_vel_enu[1];
                }
                // xy position mode
                else {
                    float d_pos_body[2] = {
                        Command_Now.pos_sp[0],
                        Command_Now.pos_sp[1]};   // the desired xy position in
                                                  // Body Frame
                    float
                        d_pos_enu[2];   // the desired xy position in enu Frame
                                        // (The origin point is the drone)
                    rotation_yaw(pos_controller.Euler_fcu[2], d_pos_body,
                                 d_pos_enu);

                    pos_sp[0] = pos_controller.pos_drone_fcu[0] + d_pos_enu[0];
                    pos_sp[1] = pos_controller.pos_drone_fcu[1] + d_pos_enu[1];
                }

                // 根据子模式设置z轴的速度或位置目标
                // z velocity mode
                if (Command_Now.sub_mode & 0b01) {
                    vel_sp[2] = Command_Now.vel_sp[2];
                }
                // z posiiton mode
                {
                    pos_sp[2] =
                        pos_controller.pos_drone_fcu[2] + Command_Now.pos_sp[2];
                }

                // 计算并设置yaw目标角度
                yaw_sp = pos_controller.Euler_fcu[2] * 180 / M_PI +
                         Command_Now.yaw_sp;
            }

            // 根据当前状态和目标点计算加速度指令
            accel_sp = pos_controller_pid.pos_controller(
                pos_controller.pos_drone_fcu, pos_controller.vel_drone_fcu,
                pos_sp, vel_sp, Command_Now.sub_mode, dt);
            // 发送加速度指令和yaw目标角度
            pos_controller.send_accel_setpoint(accel_sp, yaw_sp);
            break;

        case Hold:
            // 当前命令为保持位置，如果上一命令不是保持位置，则更新保持位置和航向
            if (Command_Last.command != Hold) {
                // 根据当前无人机在FCU坐标系下的位置，设置保持位置
                pos_controller.Hold_position =
                    Eigen::Vector3d(pos_controller.pos_drone_fcu[0],
                                    pos_controller.pos_drone_fcu[1],
                                    pos_controller.pos_drone_fcu[2]);
                // 将当前无人机的航向（欧拉角）转换为度数，作为保持航向
                pos_controller.Hold_yaw =
                    pos_controller.Euler_fcu[2] * 180 / M_PI;
                // 设置位置目标为保持位置，航向目标为保持航向
                pos_sp = pos_controller.Hold_position;
                yaw_sp = pos_controller.Hold_yaw;
            }
            // 通过位置PID控制器计算加速度设定值
            accel_sp = pos_controllerpid.pos_controller(
                pos_controller.pos_drone_fcu, pos_controller.vel_drone_fcu,
                pos_controller.Hold_position, vel_sp, Command_Now.sub_mode, dt);
            // 发送加速度设定值和保持航向给无人机
            pos_controller.send_accel_setpoint(accel_sp,
                                               pos_controller.Hold_yaw);
            break;

        case Land:   // 处理着陆命令
            // 如果当前命令不是着陆，则设置着陆点位置和目标yaw角度
            if (Command_Last.command != Land) {
                // 从控制器获取当前无人机位置和起飞位置，计算目标位置
                pos_sp = Eigen::Vector3d(pos_controller.pos_drone_fcu[0],
                                         pos_controller.pos_drone_fcu[1],
                                         pos_controller.Takeoff_position[2]);
                // 将欧拉角转换为yaw角度（度）
                yaw_sp = pos_controller.Euler_fcu[2] * 180 / M_PI;
            }

            // 检查无人机是否接近起飞高度
            // 如果距离起飞高度小于10厘米，则直接上锁并切换为手动模式；
            if (abs(pos_controller.pos_drone_fcu[2] -
                    pos_controller.Takeoff_position[2]) < (0.1)) {
                // 如果当前模式是OFFBOARD，则切换到MANUAL模式
                if (pos_controller.current_state.mode == "OFFBOARD") {
                    pos_controller.mode_cmd.request.custom_mode = "MANUAL";
                    pos_controller.set_mode_client.call(
                        pos_controller.mode_cmd);
                }

                // 如果无人机已武装，则进行解武装操作
                if (pos_controller.current_state.armed) {
                    pos_controller.arm_cmd.request.value = false;
                    pos_controller.arming_client.call(pos_controller.arm_cmd);
                }

                // 如果解武装操作成功，给出反馈
                if (pos_controller.arm_cmd.response.success) {
                    cout << "Disarm successfully!" << endl;
                }
            } else {
                // 计算着陆过程中的加速度指令
                accel_sp = pos_controller_pid.pos_controller(
                    pos_controller.pos_drone_fcu, pos_controller.vel_drone_fcu,
                    pos_sp, vel_sp, 0b00, dt);
                // 发送加速度和yaw角度指令给控制器
                pos_controller.send_accel_setpoint(accel_sp, yaw_sp);
            }

            break;

        case Disarm:
            /* 当控制器处于OFFBOARD模式时，切换到手动模式 */
            if (pos_controller.current_state.mode == "OFFBOARD") {
                pos_controller.mode_cmd.request.custom_mode = "MANUAL";
                pos_controller.set_mode_client.call(pos_controller.mode_cmd);
            }
            /* 当无人机已武装时，尝试解除武装 */
            if (pos_controller.current_state.armed) {
                pos_controller.arm_cmd.request.value = false;
                pos_controller.arming_client.call(pos_controller.arm_cmd);
            }
            /* 如果解除武装成功，给出反馈 */
            if (pos_controller.arm_cmd.response.success) {
                cout << "Disarm successfully!" << endl;
            }
            break;

        case Arm:
            /* 如果控制器不处于OFFBOARD模式，提示用户切换 */
            if (pos_controller.current_state.mode != "OFFBOARD") {
                cout << "Please switch to OFFBOARD mode!" << endl;
                break;
            }
            /* 当无人机未武装时，尝试武装 */
            if (!pos_controller.current_state.armed) {
                pos_controller.arm_cmd.request.value = true;
                pos_controller.arming_client.call(pos_controller.arm_cmd);
            }
            /* 如果武装成功，给出反馈 */
            if (pos_controller.arm_cmd.response.success) {
                cout << "Arm successfully!" << endl;
            }
            break;

        case Failsafe_land:
            /* 处理故障安全着陆逻辑 */
            break;

        case Idle:
            /* 使控制器进入空闲状态 */
            pos_controller.idle();
            break;

        case Takeoff:
            /* 计算起飞位置和速度目标 */
            pos_sp = Eigen::Vector3d(pos_controller.Takeoff_position[0],
                                     pos_controller.Takeoff_position[1],
                                     pos_controller.Takeoff_position[2] +
                                         pos_controller.Takeoff_height);
            vel_sp = Eigen::Vector3d(0.0, 0.0, 0.0);
            /* 根据当前状态和目标计算加速度指令 */
            accel_sp = pos_controller_pid.pos_controller(
                pos_controller.pos_drone_fcu, pos_controller.vel_drone_fcu,
                pos_sp, vel_sp, Command_Now.sub_mode, dt);

            /* 发送加速度设定值给控制器 */
            pos_controller.send_accel_setpoint(accel_sp, Command_Now.yaw_sp);

            break;
        }

        /* 更新上一次的命令状态 */
        Command_Last = Command_Now;

        /* 等待下一个控制周期 */
        rate.sleep();
    }
    return 0;
}

// 【获取当前时间函数】 单位：秒
float get_ros_time(ros::Time begin) {
    ros::Time time_now = ros::Time::now();
    float currTimeSec  = time_now.sec - begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

/**
 * @brief 打印当前命令状态
 *
 * 该函数用于输出当前命令的详细信息，包括命令类型、子模式以及各个轴（如X、Y、Z和yaw）的设定点。
 * 设定点可以是位置（以米为单位）或速度（以米/秒为单位），取决于子模式的配置。
 * 命令类型包括Move_ENU（以ENU坐标系移动）和Move_Body（以机身坐标系移动）。
 */
void prinft_command_state() {
    cout << ">>>>>>>>>>>Command State<<<<<<<<<<<<<<<<<" << endl;

    int sub_mode;
    sub_mode = Command_Now.sub_mode;   // 获取当前命令的子模式

    // 根据当前命令类型进行不同的处理
    switch (Command_Now.command) {
    case Move_ENU:
        cout << "Command: [ Move_ENU ] " << endl;

        // 根据xy子模式输出相应的设定点信息
        if ((sub_mode & 0b10) == 0)   // xy channel
        {
            cout << "Submode: xy position control " << endl;
            cout << "X_setpoint   : " << Command_Now.pos_sp[0] << " [ m ]"
                 << "  Y_setpoint : " << Command_Now.pos_sp[1] << " [ m ]"
                 << endl;
        } else {
            cout << "Submode: xy velocity control " << endl;
            cout << "X_setpoint   : " << Command_Now.vel_sp[0] << " [m/s]"
                 << "  Y_setpoint : " << Command_Now.vel_sp[1] << " [m/s]"
                 << endl;
        }

        // 根据z子模式输出相应的设定点信息
        if ((sub_mode & 0b01) == 0)   // z channel
        {
            cout << "Submode:  z position control " << endl;
            cout << "Z_setpoint   : " << Command_Now.pos_sp[2] << " [ m ]"
                 << endl;
        } else {
            cout << "Submode:  z velocity control " << endl;
            cout << "Z_setpoint   : " << Command_Now.vel_sp[2] << " [m/s]"
                 << endl;
        }

        cout << "Yaw_setpoint : " << Command_Now.yaw_sp << " [deg] " << endl;

        break;
    case Move_Body:
        cout << "Command: [ Move_Body ] " << endl;

        // 与Move_ENU相同，根据xy和z子模式输出设定点信息
        if ((sub_mode & 0b10) == 0)   // xy channel
        {
            cout << "Submode: xy position control " << endl;
            cout << "X_setpoint   : " << Command_Now.pos_sp[0] << " [ m ]"
                 << "  Y_setpoint : " << Command_Now.pos_sp[1] << " [ m ]"
                 << endl;
        } else {
            cout << "Submode: xy velocity control " << endl;
            cout << "X_setpoint   : " << Command_Now.vel_sp[0] << " [m/s]"
                 << "  Y_setpoint : " << Command_Now.vel_sp[1] << " [m/s]"
                 << endl;
        }

        if ((sub_mode & 0b01) == 0)   // z channel
        {
            cout << "Submode:  z position control " << endl;
            cout << "Z_setpoint   : " << Command_Now.pos_sp[2] << " [ m ]"
                 << endl;
        } else {
            cout << "Submode:  z velocity control " << endl;
            cout << "Z_setpoint   : " << Command_Now.vel_sp[2] << " [m/s]"
                 << endl;
        }

        cout << "Yaw_setpoint : " << Command_Now.yaw_sp << " [deg] " << endl;

        break;

    case Hold:
        // 输出保持当前位置的指令详情
        cout << "Command: [ Hold ] " << endl;
        cout << "Hold Position [X Y Z] : " << pos_sp[0] << " [ m ] "
             << pos_sp[1] << " [ m ] " << pos_sp[2] << " [ m ] " << endl;
        cout << "Yaw_setpoint : " << yaw_sp << " [deg] " << endl;
        break;

    case Land:
        // 输出着陆指令详情，包括目标位置和yaw设定值
        cout << "Command: [ Land ] " << endl;
        cout << "Land Position [X Y Z] : " << pos_sp[0] << " [ m ] "
             << pos_sp[1] << " [ m ] " << pos_sp[2] << " [ m ] " << endl;
        cout << "Yaw_setpoint : " << Command_Now.yaw_sp << " [deg] " << endl;
        break;

    case Disarm:
        // 输出解除武装指令的确认信息
        cout << "Command: [ Disarm ] " << endl;
        break;

    case Failsafe_land:
        // 输出安全着陆指令的确认信息
        cout << "Command: [ Failsafe_land ] " << endl;
        break;

    case Idle:
        // 输出闲置指令的确认信息
        cout << "Command: [ Idle ] " << endl;
        break;

    case Takeoff:
        // 输出起飞指令详情，包括起飞位置和yaw设定值
        cout << "Command: [ Takeoff ] " << endl;
        cout << "Takeoff Position [X Y Z] : " << pos_sp[0] << " [ m ] "
             << pos_sp[1] << " [ m ] " << pos_sp[2] << " [ m ] " << endl;
        cout << "Yaw_setpoint : " << Command_Now.yaw_sp << " [deg] " << endl;
        break;
    }
}

// 【坐标系旋转函数】- 机体系到enu系
// input是机体系,output是惯性系，yaw_angle是当前偏航角
void rotation_yaw(float yaw_angle, float input[2], float output[2]) {
    output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
    output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}
