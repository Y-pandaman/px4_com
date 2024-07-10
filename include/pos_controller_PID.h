#ifndef POS_CONTROLLER_PID_H
#define POS_CONTROLLER_PID_H
#include <Eigen/Eigen>
#include <math.h>
#include <math_utils.h>

using namespace std;

namespace namespace_PID {
    class pos_controller_PID {
    public:
        /**
         * @brief 构造函数初始化位置控制器PID参数
         *
         * 该构造函数主要用于从ROS参数服务器加载位置控制器（PID）的参数。
         * 它还初始化一些状态变量和订阅器，用于接收无人机的状态信息。
         *
         * @param None
         * @return None
         */
        pos_controller_PID(void) : pos_pid_nh("~") {
            // 从ROS参数服务器加载位置控制（PID）参数
            pos_pid_nh.param<float>("MPC_XY_P", MPC_XY_P, 1.0);
            pos_pid_nh.param<float>("MPC_Z_P", MPC_Z_P, 1.0);
            pos_pid_nh.param<float>("MPC_XY_VEL_P", MPC_XY_VEL_P, 0.1);
            pos_pid_nh.param<float>("MPC_Z_VEL_P", MPC_Z_VEL_P, 0.1);
            pos_pid_nh.param<float>("MPC_XY_VEL_I", MPC_XY_VEL_I, 0.02);
            pos_pid_nh.param<float>("MPC_Z_VEL_I", MPC_Z_VEL_I, 0.02);
            pos_pid_nh.param<float>("MPC_XY_VEL_D", MPC_XY_VEL_D, 0.01);
            pos_pid_nh.param<float>("MPC_Z_VEL_D", MPC_Z_VEL_D, 0.01);
            pos_pid_nh.param<float>("MPC_XY_VEL_MAX", MPC_XY_VEL_MAX, 1.0);
            pos_pid_nh.param<float>("MPC_Z_VEL_MAX", MPC_Z_VEL_MAX, 0.5);
            pos_pid_nh.param<float>("MPC_THRUST_HOVER", MPC_THRUST_HOVER, 0.4);
            pos_pid_nh.param<float>("MPC_THR_MIN", MPC_THR_MIN, 0.1);
            pos_pid_nh.param<float>("MPC_THR_MAX", MPC_THR_MAX, 0.9);
            pos_pid_nh.param<float>("tilt_max", tilt_max, 5.0);
            pos_pid_nh.param<float>("MPC_VELD_LP", MPC_VELD_LP, 5.0);

            // 初始化无人机状态变量
            pos_drone          = Eigen::Vector3d(0.0, 0.0, 0.0);
            vel_drone          = Eigen::Vector3d(0.0, 0.0, 0.0);
            vel_setpoint       = Eigen::Vector3d(0.0, 0.0, 0.0);
            thrust_sp          = Eigen::Vector3d(0.0, 0.0, 0.0);
            vel_P_output       = Eigen::Vector3d(0.0, 0.0, 0.0);
            thurst_int         = Eigen::Vector3d(0.0, 0.0, 0.0);
            vel_D_output       = Eigen::Vector3d(0.0, 0.0, 0.0);
            error_vel_dot_last = Eigen::Vector3d(0.0, 0.0, 0.0);
            error_vel_last     = Eigen::Vector3d(0.0, 0.0, 0.0);
            delta_time         = 0.02;
            flag_offboard      = 0;

            // 订阅无人机状态信息
            state_sub = pos_pid_nh.subscribe<mavros_msgs::State>(
                "/mavros/state", 10, &pos_controller_PID::state_cb, this);
        }

        // PID parameter for the control law
        float MPC_XY_P;
        float MPC_Z_P;
        float MPC_XY_VEL_P;
        float MPC_Z_VEL_P;
        float MPC_XY_VEL_I;
        float MPC_Z_VEL_I;
        float MPC_XY_VEL_D;
        float MPC_Z_VEL_D;

        // Limitation of the velocity
        float MPC_XY_VEL_MAX;
        float MPC_Z_VEL_MAX;

        // Hover thrust of drone (decided by the mass of the drone)
        float MPC_THRUST_HOVER;

        // Limitation of the thrust
        float MPC_THR_MIN;
        float MPC_THR_MAX;

        // Limitation of the tilt angle (roll and pitch)  [degree]
        float tilt_max;

        // Current position and velocity of the drone
        Eigen::Vector3d pos_drone;
        Eigen::Vector3d vel_drone;

        // Desired position and velocity of the drone
        Eigen::Vector3d vel_setpoint;

        // Desired thurst of the drone[the output of this class]
        Eigen::Vector3d thrust_sp;

        // Output of the vel loop in PID [thurst_int is the I]
        Eigen::Vector3d vel_P_output;
        Eigen::Vector3d thurst_int;
        Eigen::Vector3d vel_D_output;

        float MPC_VELD_LP;

        // The delta time between now and the last step
        float delta_time;

        // Derriv of the velocity error in last step [used for the D-output in
        // vel loop]
        Eigen::Vector3d error_vel_dot_last;
        Eigen::Vector3d error_vel_last;

        // Current state of the drone
        mavros_msgs::State current_state;

        // Flag of the offboard mode [1 for OFFBOARD mode , 0 for non-OFFBOARD
        // mode]
        int flag_offboard;

        // Output of thrustToAttitude
        Eigen::Vector3d euler_sp;

        // Printf the PID parameter
        void printf_param();

        // Printf the control result
        void printf_result();

        // Position control main function [Input: current pos, current vel,
        // desired state(pos or vel), sub_mode, time_now; Output: desired
        // thrust;]
        Eigen::Vector3d pos_controller(Eigen::Vector3d pos, Eigen::Vector3d vel,
                                       Eigen::Vector3d pos_sp,
                                       Eigen::Vector3d vel_sp, int sub_mode,
                                       float curtime);

        // Position control loop [Input: current pos, desired pos; Output:
        // desired vel]
        void _positionController(Eigen::Vector3d pos_sp, Eigen::Vector3d vel_sp,
                                 int sub_mode);

        // Velocity control loop [Input: current vel, desired vel; Output:
        // desired thrust]
        void _velocityController();

        Eigen::Vector3d cal_vel_error_deriv(Eigen::Vector3d error_now);

    private:
        ros::NodeHandle pos_pid_nh;

        ros::Subscriber state_sub;

        void state_cb(const mavros_msgs::State::ConstPtr& msg) {
            current_state = *msg;

            if (current_state.mode == "OFFBOARD") {
                flag_offboard = 1;
            } else {
                flag_offboard = 0;
            }
        }
    };

    /**
     * @brief 位置控制器（PID控制器）
     *
     * 该函数实现了对无人机位置的控制，通过比较当前位置和速度与目标位置和速度，
     * 计算出推力指令，以驱动无人机达到目标位置。
     *
     * @param pos 当前无人机的位置。
     * @param vel 当前无人机的速度。
     * @param pos_sp 无人机的目标位置。
     * @param vel_sp 无人机的目标速度。
     * @param sub_mode 控制器的子模式，用于实现不同的控制策略。
     * @param dt 时间步长，用于更新控制器的状态。
     * @return 返回推力指令，用于控制无人机的运动。
     */
    Eigen::Vector3d pos_controller_PID::pos_controller(Eigen::Vector3d pos,
                                                       Eigen::Vector3d vel,
                                                       Eigen::Vector3d pos_sp,
                                                       Eigen::Vector3d vel_sp,
                                                       int sub_mode, float dt) {
        // 更新当前无人机的位置和速度
        pos_drone = pos;
        vel_drone = vel;

        // 设置时间步长
        delta_time = dt;

        // 调用位置控制器，计算位置误差和速度误差
        _positionController(pos_sp, vel_sp, sub_mode);

        // 调用速度控制器，根据位置和速度误差计算推力指令
        _velocityController();

        // 返回计算得到的推力指令
        return thrust_sp;
    }

    /**
     * @brief 位置控制器PID算法实现。
     *
     * 此函数根据位置设定值、当前无人机位置以及控制模式参数来计算速度设定值，
     * 控制模式参数决定控制目标（位置或速度）以及控制轴（XY平面或Z轴）。
     *
     * @param pos_sp 三维空间中的位置设定值。
     * @param vel_sp 三维空间中的速度设定值。
     * @param sub_mode
     * 控制模式，二进制值，用于确定控制目标（位置或速度）和控制轴（XY或Z）。
     */
    void pos_controller_PID::_positionController(Eigen::Vector3d pos_sp,
                                                 Eigen::Vector3d vel_sp,
                                                 int sub_mode) {
        //# sub_mode 2-bit value:
        //# 0 for position, 1 for vel, 1st for xy, 2nd for z.
        //#                   xy position     xy velocity
        //# z position       	0b00(0)       0b10(2)
        //# z velocity		0b01(1)       0b11(3)

        // 检查XY轴是否处于位置控制模式
        if ((sub_mode & 0b10) == 0)   // XY通道
        {
            // 根据位置误差和位置P增益计算XY轴的速度设定值
            vel_setpoint(0) = MPC_XY_P * (pos_sp(0) - pos_drone(0));
            vel_setpoint(1) = MPC_XY_P * (pos_sp(1) - pos_drone(1));
        } else {
            // 使用给定的速度设定值作为XY轴的速度设定值
            vel_setpoint(0) = vel_sp(0);
            vel_setpoint(1) = vel_sp(1);
        }

        // 检查Z轴是否处于位置控制模式
        if ((sub_mode & 0b01) == 0)   // Z通道
        {
            // 根据位置误差和位置P增益计算Z轴的速度设定值
            vel_setpoint(2) = MPC_Z_P * (pos_sp(2) - pos_drone(2));
        } else {
            // 使用给定的速度设定值作为Z轴的速度设定值
            vel_setpoint(2) = vel_sp(2);
        }

        // 对速度设定值进行限制，确保其不超过最大值
        // 限制速度设定值
        vel_setpoint(0) = constrain_function2(vel_setpoint(0), -MPC_XY_VEL_MAX,
                                              MPC_XY_VEL_MAX);
        vel_setpoint(1) = constrain_function2(vel_setpoint(1), -MPC_XY_VEL_MAX,
                                              MPC_XY_VEL_MAX);
        vel_setpoint(2) =
            constrain_function2(vel_setpoint(2), -MPC_Z_VEL_MAX, MPC_Z_VEL_MAX);
    }

    /**
     * @brief 无人机的位置控制PID函数。
     * 此函数主要根据速度误差计算X、Y和Z方向所需的推力，并对积分和微分项应用反风偏和饱和限制，
     * 以确保飞行控制的稳定性。
     */
    void pos_controller_PID::_velocityController() {
        // 计算速度误差
        Eigen::Vector3d error_vel = vel_setpoint - vel_drone;

        // 计算X、Y和Z方向的比例输出
        vel_P_output(0) = MPC_XY_VEL_P * error_vel(0);
        vel_P_output(1) = MPC_XY_VEL_P * error_vel(1);
        vel_P_output(2) = MPC_Z_VEL_P * error_vel(2);

        // 计算速度误差的导数
        Eigen::Vector3d vel_error_deriv = cal_vel_error_deriv(error_vel);

        // 计算X、Y和Z方向的微分输出
        vel_D_output(0) = MPC_XY_VEL_D * vel_error_deriv(0);
        vel_D_output(1) = MPC_XY_VEL_D * vel_error_deriv(1);
        vel_D_output(2) = MPC_Z_VEL_D * vel_error_deriv(2);

        // 计算Z方向的期望推力，考虑悬停推力
        float thrust_desired_Z = vel_P_output(2) + thurst_int(2) +
                                 vel_D_output(2) + MPC_THRUST_HOVER;

        // 在Z方向应用反风偏，防止积分项在推力饱和时造成不稳定
        // 反风偏应用于Z方向。
        // 两种情况：期望推力大于最大推力，且速度误差正向；期望推力小于最小推力，且速度误差负向
        bool stop_integral_Z =
            (thrust_desired_Z >= MPC_THR_MAX && error_vel(2) >= 0.0f) ||
            (thrust_desired_Z <= MPC_THR_MIN && error_vel(2) <= 0.0f);
        if (!stop_integral_Z) {
            thurst_int(2) += MPC_Z_VEL_I * error_vel(2) * delta_time;

            // 限制推力积分，防止推力过大
            thurst_int(2) = min(fabs(thurst_int(2)), MPC_THR_MAX) *
                            sign_function(thurst_int(2));
        }

        // 饱和Z方向的推力设定点，防止超出最大或最小推力
        thrust_sp(2) =
            constrain_function2(thrust_desired_Z, MPC_THR_MIN, MPC_THR_MAX);

        // 计算X和Y方向的期望推力
        float thrust_desired_X;
        float thrust_desired_Y;
        thrust_desired_X = vel_P_output(0) + thurst_int(0) + vel_D_output(0);
        thrust_desired_Y = vel_P_output(1) + thurst_int(1) + vel_D_output(1);

        // 根据倾斜角度和额外推力确定X和Y方向的最大允许推力
        float thrust_max_XY_tilt =
            fabs(thrust_sp(2)) * tanf(tilt_max / 180.0 * M_PI);
        float thrust_max_XY =
            sqrtf(MPC_THR_MAX * MPC_THR_MAX - thrust_sp(2) * thrust_sp(2));
        thrust_max_XY = min(thrust_max_XY_tilt, thrust_max_XY);

        // 饱和X和Y方向的推力，防止超出最大推力
        thrust_sp(0) = thrust_desired_X;
        thrust_sp(1) = thrust_desired_Y;

        // 如果计算出的推力超过最大允许值，则调整至最大值
        if ((thrust_desired_X * thrust_desired_X +
             thrust_desired_Y * thrust_desired_Y) >
            thrust_max_XY * thrust_max_XY) {
            float mag    = sqrtf((thrust_desired_X * thrust_desired_X +
                               thrust_desired_Y * thrust_desired_Y));
            thrust_sp(0) = thrust_desired_X / mag * thrust_max_XY;
            thrust_sp(1) = thrust_desired_Y / mag * thrust_max_XY;
        }

        // 在X和Y方向应用反风偏以提高饱和期间的控制性能
        float arw_gain = 2.f / MPC_XY_VEL_P;

        float vel_err_lim_x, vel_err_lim_y;
        vel_err_lim_x =
            error_vel(0) - (thrust_desired_X - thrust_sp(0)) * arw_gain;
        vel_err_lim_y =
            error_vel(1) - (thrust_desired_Y - thrust_sp(1)) * arw_gain;

        // 更新X和Y方向的积分项
        thurst_int(0) += MPC_XY_VEL_I * vel_err_lim_x * delta_time;
        thurst_int(1) += MPC_XY_VEL_I * vel_err_lim_y * delta_time;

        // 如果不在OFFBOARD模式下，将所有积分项重置为零
        if (flag_offboard == 0) {
            thurst_int = Eigen::Vector3d(0.0, 0.0, 0.0);
        }
    }

    /**
     * @brief 计算速度误差导数的PID控制器
     *
     * 该函数根据当前速度误差和上一时刻的速度误差，计算出速度误差导数的PID控制输出。
     * 使用了低通滤波器来平滑控制输出，减少控制系统的波动。
     *
     * @param error_now 当前速度误差
     * @return Eigen::Vector3d PID控制输出
     */
    Eigen::Vector3d
    pos_controller_PID::cal_vel_error_deriv(Eigen::Vector3d error_now) {
        Eigen::Vector3d error_vel_dot_now;   // 当前速度误差的导数
        // 计算速度误差的导数，利用差分近似
        error_vel_dot_now = (error_now - error_vel_last) / delta_time;

        // 更新上一时刻的速度误差为当前误差
        error_vel_last = error_now;

        // 低通滤波器参数初始化
        float a, b;
        b = 2 * M_PI * MPC_VELD_LP * delta_time;   // 滤波器截止频率的计算
        a = b / (1 + b);                           // 滤波器增益的计算

        Eigen::Vector3d output;   // 控制输出

        // 计算PID控制输出，结合了误差导数的当前值和上一控制输出的滤波值
        output = a * error_vel_dot_now + (1 - a) * error_vel_dot_last;

        // 更新上一控制输出为当前计算的输出
        error_vel_dot_last = output;

        return output;
    }

    void pos_controller_PID::printf_result() {
        cout << ">>>>>>>>>>>>>>>Position Controller<<<<<<<<<<<<<<<<<" << endl;

        //固定的浮点显示
        cout.setf(ios::fixed);
        //左对齐
        cout.setf(ios::left);
        // 强制显示小数点
        cout.setf(ios::showpoint);
        // 强制显示符号
        cout.setf(ios::showpos);

        cout << setprecision(2);

        cout << "Velocity_sp  [X Y Z] : " << vel_setpoint[0] << " [m/s] "
             << vel_setpoint[1] << " [m/s] " << vel_setpoint[2] << " [m/s] "
             << endl;
        cout << "thrust_sp    [X Y Z] : " << thrust_sp[0] << " [m/s^2] "
             << thrust_sp[1] << " [m/s^2] " << thrust_sp[2] << " [m/s^2] "
             << endl;
    }

    // 【打印参数函数】
    void pos_controller_PID::printf_param() {
        cout << ">>>>>>>>>>>PID Parameter<<<<<<<<<<<<<<<<" << endl;

        cout << "Position Loop:  " << endl;
        cout << "MPC_XY_P : " << MPC_XY_P << endl;
        cout << "MPC_Z_P : " << MPC_Z_P << endl;
        cout << "Velocity Loop:  " << endl;
        cout << "MPC_XY_VEL_P : " << MPC_XY_VEL_P << endl;
        cout << "MPC_Z_VEL_P : " << MPC_Z_VEL_P << endl;
        cout << "MPC_XY_VEL_I : " << MPC_XY_VEL_I << endl;
        cout << "MPC_Z_VEL_I : " << MPC_Z_VEL_I << endl;
        cout << "MPC_XY_VEL_D : " << MPC_XY_VEL_D << endl;
        cout << "MPC_Z_VEL_D : " << MPC_Z_VEL_D << endl;

        cout << "Limit:  " << endl;
        cout << "MPC_XY_VEL_MAX : " << MPC_XY_VEL_MAX << endl;
        cout << "MPC_Z_VEL_MAX : " << MPC_Z_VEL_MAX << endl;

        cout << "tilt_max : " << tilt_max << endl;
        cout << "MPC_THR_MIN : " << MPC_THR_MIN << endl;
        cout << "MPC_THR_MAX : " << MPC_THR_MAX << endl;
        cout << "MPC_THRUST_HOVER : " << MPC_THRUST_HOVER << endl;
    }

}   // namespace namespace_PID
#endif
