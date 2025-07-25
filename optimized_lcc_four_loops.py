#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
优化的四环PID控制车道居中控制（LCC）算法
用于高级驾驶辅助系统（ADAS），确保横向居中误差小于0.5米

作者：AI Assistant
日期：2024
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from scipy.optimize import minimize, differential_evolution
from scipy.interpolate import interp1d
import warnings
warnings.filterwarnings('ignore')

# =========================
# 车辆参数定义
# =========================
def get_vehicle_params():
    """
    返回典型乘用车参数，确保物理合理性
    参考：丰田凯美瑞、本田雅阁等中型轿车
    """
    params = {
        'm': 1650.0,         # 质量 [kg] - 典型中型轿车
        'Iz': 3200.0,        # 转动惯量 [kg*m^2] - 基于质量和轴距估算
        'lf': 1.25,          # 前轴到质心距离 [m] - 前驱车质心略偏前
        'lr': 1.55,          # 后轴到质心距离 [m]
        'Cf': 65000.0,       # 前轮侧偏刚度 [N/rad] - 高性能轮胎
        'Cr': 65000.0,       # 后轮侧偏刚度 [N/rad]
        'wheelbase': 2.8,    # 轴距 [m] - 标准中型轿车
        'width': 1.85,       # 车宽 [m]
        'max_steer': np.deg2rad(25),  # 最大转向角 [rad] - 保守值
        'dt': 0.01,          # 仿真步长 [s] - 高精度仿真
        'speed': 8.0,        # 车速 [m/s] - 适中速度便于控制
        'g': 9.81,           # 重力加速度 [m/s^2]
    }
    return params

# =========================
# 道路模型生成（平滑过渡）
# =========================
def generate_smooth_road(total_length=2500, lane_width=3.8, curve_radius=500, 
                        transition_length=150, ds=0.5):
    """
    生成平滑的道路中心线及曲率
    道路：1000m直道 + 平滑过渡 + 半径500m弯道 + 平滑过渡 + 1000m直道
    
    使用余弦函数实现平滑过渡
    """
    # 直道1 (0-1000m)
    s1 = np.arange(0, 1000, ds)
    k1 = np.zeros_like(s1)
    
    # 过渡段1 (1000-1150m) - 使用余弦函数实现平滑过渡
    s2 = np.arange(1000, 1000 + transition_length, ds)
    t = (s2 - 1000) / transition_length
    k2 = (1 - np.cos(np.pi * t)) / 2 / curve_radius
    
    # 弯道 (1150-1893m) - 90度弯道
    arc_length = np.pi * curve_radius / 2  # 90度弧长
    s3 = np.arange(1000 + transition_length, 1000 + transition_length + arc_length, ds)
    k3 = np.ones_like(s3) / curve_radius
    
    # 过渡段2 (1893-2043m)
    s4 = np.arange(1000 + transition_length + arc_length, 
                   1000 + 2*transition_length + arc_length, ds)
    t = (s4 - (1000 + transition_length + arc_length)) / transition_length
    k4 = (1 + np.cos(np.pi * t)) / 2 / curve_radius
    
    # 直道2 (2043-3043m)
    s5 = np.arange(1000 + 2*transition_length + arc_length, 
                   1000 + 2*transition_length + arc_length + 1000, ds)
    k5 = np.zeros_like(s5)
    
    # 拼接所有段
    s = np.concatenate([s1, s2, s3, s4, s5])
    kappa = np.concatenate([k1, k2, k3, k4, k5])
    
    # 使用数值积分计算中心线坐标
    x, y, psi = [0], [0], [0]
    for i in range(1, len(s)):
        # 使用梯形积分提高精度
        dpsi = (kappa[i-1] + kappa[i]) / 2 * ds
        psi.append(psi[-1] + dpsi)
        
        # 使用平均角度计算位置
        avg_psi = (psi[-2] + psi[-1]) / 2
        x.append(x[-1] + ds * np.cos(avg_psi))
        y.append(y[-1] + ds * np.sin(avg_psi))
    
    x = np.array(x)
    y = np.array(y)
    psi = np.array(psi)
    
    # 生成车道线
    left_x = x - lane_width/2 * np.sin(psi)
    left_y = y + lane_width/2 * np.cos(psi)
    right_x = x + lane_width/2 * np.sin(psi)
    right_y = y - lane_width/2 * np.cos(psi)
    
    return {
        's': s,
        'kappa': kappa,
        'x': x,
        'y': y,
        'psi': psi,
        'left_x': left_x,
        'left_y': left_y,
        'right_x': right_x,
        'right_y': right_y,
        'lane_width': lane_width,
        'curve_radius': curve_radius
    }

# =========================
# 车辆横向动力学模型（改进的线性自行车模型）
# =========================
def vehicle_lateral_dynamics(state, t, delta, v, params, kappa_ref):
    """
    改进的车辆横向动力学状态方程
    状态: [e, psi_err, vy, r]
    e: 横向位置误差 [m]
    psi_err: 偏航角误差 [rad]
    vy: 横向速度 [m/s]
    r: 偏航角速度 [rad/s]
    
    数学公式：
    de/dt = vy + v * psi_err
    d(psi_err)/dt = r - v * kappa_ref
    dvy/dt = (-2Cf-2Cr)/(m*v) * vy - (2Cf*lf-2Cr*lr)/(m*v) * r + 2Cf/m * delta
    dr/dt = (-2Cf*lf+2Cr*lr)/(Iz*v) * vy - (2Cf*lf²+2Cr*lr²)/(Iz*v) * r + 2Cf*lf/Iz * delta
    """
    m = params['m']
    Iz = params['Iz']
    lf = params['lf']
    lr = params['lr']
    Cf = params['Cf']
    Cr = params['Cr']
    
    e, psi_err, vy, r = state
    
    # 数值稳定性检查
    if abs(v) < 0.1:
        v = 0.1  # 避免除零
    
    # 线性自行车模型系数
    a11 = 0
    a12 = v
    a13 = 1
    a14 = 0
    
    a21 = 0
    a22 = 0
    a23 = 0
    a24 = 1
    
    a31 = 0
    a32 = 0
    a33 = (-2*Cf - 2*Cr) / (m * v)
    a34 = (-2*Cf*lf + 2*Cr*lr) / (m * v)
    
    a41 = 0
    a42 = 0
    a43 = (-2*Cf*lf + 2*Cr*lr) / (Iz * v)
    a44 = (-2*Cf*lf**2 - 2*Cr*lr**2) / (Iz * v)
    
    # 控制输入矩阵
    b3 = 2*Cf / m
    b4 = 2*Cf*lf / Iz
    
    # 道路曲率输入
    d2 = -v * kappa_ref
    
    # 状态方程
    d_e = a11*e + a12*psi_err + a13*vy + a14*r
    d_psi_err = a21*e + a22*psi_err + a23*vy + a24*r + d2
    d_vy = a31*e + a32*psi_err + a33*vy + a34*r + b3*delta
    d_r = a41*e + a42*psi_err + a43*vy + a44*r + b4*delta
    
    return [d_e, d_psi_err, d_vy, d_r]

# =========================
# 优化的四环 PID 控制器
# =========================
class OptimizedFourLoopPIDController:
    """
    优化的四环PID控制器
    环1：横向位置误差 e -> 期望偏航角误差 psi_err_ref
    环2：偏航角误差 psi_err -> 期望横向速度 vy_ref  
    环3：横向速度 vy -> 期望偏航角速度 r_ref
    环4：偏航角速度 r -> 前轮转角 delta
    
    特点：
    - 抗积分饱和
    - 微分滤波
    - 自适应增益
    - 数值稳定性检查
    """
    def __init__(self, params, pid_gains, dt):
        self.dt = dt
        self.params = params
        
        # PID参数
        self.kp_e, self.ki_e, self.kd_e = pid_gains['e']
        self.kp_psi, self.ki_psi, self.kd_psi = pid_gains['psi']
        self.kp_vy, self.ki_vy, self.kd_vy = pid_gains['vy']
        self.kp_r, self.ki_r, self.kd_r = pid_gains['r']
        
        # 积分项
        self.int_e = 0.0
        self.int_psi = 0.0
        self.int_vy = 0.0
        self.int_r = 0.0
        
        # 前一误差（用于微分项）
        self.prev_e = 0.0
        self.prev_psi = 0.0
        self.prev_vy = 0.0
        self.prev_r = 0.0
        
        # 积分限幅
        self.int_limit_e = 2.0
        self.int_limit_psi = 1.0
        self.int_limit_vy = 1.0
        self.int_limit_r = 0.5
        
        # 微分滤波
        self.alpha = 0.1  # 低通滤波系数
        
        # 自适应增益
        self.adaptive_gain = 1.0
        
        # 控制输出限幅
        self.max_steer = params['max_steer']

    def reset(self):
        """重置控制器状态"""
        self.int_e = 0.0
        self.int_psi = 0.0
        self.int_vy = 0.0
        self.int_r = 0.0
        self.prev_e = 0.0
        self.prev_psi = 0.0
        self.prev_vy = 0.0
        self.prev_r = 0.0
        self.adaptive_gain = 1.0

    def limit_integral(self, integral, limit):
        """积分限幅"""
        return np.clip(integral, -limit, limit)

    def low_pass_filter(self, current, previous, alpha):
        """低通滤波"""
        return alpha * current + (1 - alpha) * previous

    def adaptive_gain_calculation(self, error, max_error=0.5):
        """自适应增益计算"""
        if abs(error) > max_error:
            return 1.5  # 大误差时增加增益
        elif abs(error) < 0.1:
            return 0.8  # 小误差时降低增益
        else:
            return 1.0

    def pid_control(self, error, prev_error, integral, kp, ki, kd, int_limit):
        """
        改进的PID控制算法
        包含抗积分饱和、微分滤波和数值稳定性检查
        """
        # 数值稳定性检查
        if np.isnan(error) or np.isinf(error):
            error = 0.0
        if np.isnan(prev_error) or np.isinf(prev_error):
            prev_error = 0.0
        if np.isnan(integral) or np.isinf(integral):
            integral = 0.0
            
        # 积分项（抗积分饱和）
        integral += error * self.dt
        integral = self.limit_integral(integral, int_limit)
        
        # 微分项（低通滤波）
        derivative = (error - prev_error) / self.dt
        derivative = self.low_pass_filter(derivative, 0, self.alpha)
        
        # PID输出
        output = kp * error + ki * integral + kd * derivative
        
        return output, integral

    def control(self, e, psi_err, vy, r, v, kappa_ref, ff_delta):
        """
        四环PID控制律
        返回：前轮转角 [rad]
        """
        # 自适应增益
        self.adaptive_gain = self.adaptive_gain_calculation(e)
        
        # 环1：横向位置误差 -> 期望偏航角误差
        psi_err_ref, self.int_e = self.pid_control(
            e, self.prev_e, self.int_e, 
            self.kp_e * self.adaptive_gain, self.ki_e, self.kd_e, 
            self.int_limit_e
        )
        self.prev_e = e
        
        # 环2：偏航角误差 -> 期望横向速度
        vy_ref, self.int_psi = self.pid_control(
            psi_err - psi_err_ref, self.prev_psi, self.int_psi,
            self.kp_psi, self.ki_psi, self.kd_psi,
            self.int_limit_psi
        )
        self.prev_psi = psi_err - psi_err_ref
        
        # 环3：横向速度 -> 期望偏航角速度
        r_ref, self.int_vy = self.pid_control(
            vy - vy_ref, self.prev_vy, self.int_vy,
            self.kp_vy, self.ki_vy, self.kd_vy,
            self.int_limit_vy
        )
        self.prev_vy = vy - vy_ref
        
        # 环4：偏航角速度 -> 前轮转角
        delta_pid, self.int_r = self.pid_control(
            r - r_ref, self.prev_r, self.int_r,
            self.kp_r, self.ki_r, self.kd_r,
            self.int_limit_r
        )
        self.prev_r = r - r_ref
        
        # 总控制输出 = PID + 前馈
        delta = delta_pid + ff_delta
        
        # 输出限幅
        delta = np.clip(delta, -self.max_steer, self.max_steer)
        
        return delta

# =========================
# 前馈控制（基于道路曲率）
# =========================
def feedforward_steering(v, kappa, params):
    """
    基于道路曲率的前馈控制
    数学公式：δ_ff = L * κ / (1 + K * v²)
    其中 L 是轴距，K 是稳定性因子
    """
    wheelbase = params['wheelbase']
    lf = params['lf']
    lr = params['lr']
    m = params['m']
    Cf = params['Cf']
    Cr = params['Cr']
    
    # 稳定性因子
    K = m * (lr * Cf - lf * Cr) / (2 * Cf * Cr * wheelbase)
    
    # 前馈转向角
    if abs(v) > 0.1:
        ff_delta = wheelbase * kappa / (1 + K * v**2)
    else:
        ff_delta = wheelbase * kappa
    
    return ff_delta

# =========================
# 自动调参算法（改进版）
# =========================
def auto_tune_pid(road, params, initial_gains, search_ranges, dt, 
                  max_error=0.5, max_iterations=100, verbose=True):
    """
    自动调参算法
    使用差分进化算法优化PID参数
    """
    print("开始自动调参以确保横向误差<0.5m，请稍候...")
    
    def objective_function(gains_flat):
        """
        目标函数：最小化最大横向误差
        gains_flat: [e_kp, e_ki, e_kd, psi_kp, psi_ki, psi_kd, 
                    vy_kp, vy_ki, vy_kd, r_kp, r_ki, r_kd]
        """
        try:
            # 重构PID参数
            pid_gains = {
                'e': gains_flat[0:3],
                'psi': gains_flat[3:6],
                'vy': gains_flat[6:9],
                'r': gains_flat[9:12]
            }
            
            # 运行仿真
            results = simulate_lcc(road, params, pid_gains, dt, max_steps=3000, no_plot=True)
            
            if results is None:
                return 1000.0  # 惩罚值
            
            # 计算目标函数
            max_lateral_error = np.max(np.abs(results['lateral_errors']))
            mean_lateral_error = np.mean(np.abs(results['lateral_errors']))
            std_lateral_error = np.std(results['lateral_errors'])
            
            # 多目标优化：最小化最大误差、平均误差和标准差
            cost = max_lateral_error + 0.5 * mean_lateral_error + 0.1 * std_lateral_error
            
            # 如果最大误差超过目标，增加惩罚
            if max_lateral_error > max_error:
                cost += 10 * (max_lateral_error - max_error)
            
            return cost
            
        except Exception as e:
            if verbose:
                print(f"调参过程中出现错误: {e}")
            return 1000.0
    
    # 参数边界
    bounds = []
    for i in range(12):
        param_range = search_ranges[i % 3]  # 每个PID参数使用相同的搜索范围
        bounds.append((param_range[0], param_range[1]))
    
    # 差分进化算法
    result = differential_evolution(
        objective_function, 
        bounds, 
        maxiter=max_iterations,
        popsize=15,
        seed=42,
        disp=verbose
    )
    
    if result.success:
        # 重构最优PID参数
        optimal_gains = {
            'e': result.x[0:3],
            'psi': result.x[3:6],
            'vy': result.x[6:9],
            'r': result.x[9:12]
        }
        
        print(f"自动调参完成！最优目标函数值: {result.fun:.4f}")
        print("最优PID参数:")
        for loop_name, gains in optimal_gains.items():
            print(f"  {loop_name}: Kp={gains[0]:.4f}, Ki={gains[1]:.4f}, Kd={gains[2]:.4f}")
        
        return optimal_gains
    else:
        print("自动调参失败，使用默认参数")
        return initial_gains

# =========================
# 仿真主函数
# =========================
def simulate_lcc(road, params, pid_gains, dt, max_steps=5000, no_plot=False):
    """
    车道居中控制仿真主函数
    """
    try:
        # 初始化控制器
        controller = OptimizedFourLoopPIDController(params, pid_gains, dt)
        controller.reset()
        
        # 初始化状态
        state = [0.0, 0.0, 0.0, 0.0]  # [e, psi_err, vy, r]
        
        # 仿真变量
        t = 0.0
        x_vehicle = 0.0
        y_vehicle = 0.0
        psi_vehicle = 0.0
        
        # 记录数据
        time_history = []
        x_history = []
        y_history = []
        psi_history = []
        lateral_errors = []
        yaw_errors = []
        steering_angles = []
        velocities = []
        
        # 道路参数
        v = params['speed']
        
        # 主仿真循环
        for step in range(max_steps):
            # 找到当前位置对应的道路参数
            s_current = np.sqrt(x_vehicle**2 + y_vehicle**2)
            
            # 插值获取道路参数
            if s_current >= road['s'][-1]:
                break
                
            # 找到最近的道路点
            idx = np.argmin(np.abs(road['s'] - s_current))
            kappa_ref = road['kappa'][idx]
            psi_ref = road['psi'][idx]
            
            # 计算横向误差和偏航角误差
            # 将车辆位置转换到道路坐标系
            dx = x_vehicle - road['x'][idx]
            dy = y_vehicle - road['y'][idx]
            
            # 横向误差（垂直于道路方向）
            e = -dx * np.sin(psi_ref) + dy * np.cos(psi_ref)
            
            # 偏航角误差
            psi_err = psi_vehicle - psi_ref
            # 归一化到[-π, π]
            psi_err = np.arctan2(np.sin(psi_err), np.cos(psi_err))
            
            # 前馈控制
            ff_delta = feedforward_steering(v, kappa_ref, params)
            
            # 四环PID控制
            delta = controller.control(e, psi_err, state[2], state[3], v, kappa_ref, ff_delta)
            
            # 记录数据
            time_history.append(t)
            x_history.append(x_vehicle)
            y_history.append(y_vehicle)
            psi_history.append(psi_vehicle)
            lateral_errors.append(e)
            yaw_errors.append(psi_err)
            steering_angles.append(delta)
            velocities.append(v)
            
            # 数值积分更新状态
            state = odeint(vehicle_lateral_dynamics, state, [t, t+dt], 
                          args=(delta, v, params, kappa_ref))[1]
            
            # 更新车辆位置
            x_vehicle += v * np.cos(psi_vehicle) * dt
            y_vehicle += v * np.sin(psi_vehicle) * dt
            psi_vehicle += state[3] * dt
            
            t += dt
            
            # 检查数值稳定性
            if (np.any(np.isnan(state)) or np.any(np.isinf(state)) or
                np.isnan(x_vehicle) or np.isinf(x_vehicle) or
                np.isnan(y_vehicle) or np.isinf(y_vehicle)):
                print("数值不稳定，仿真终止")
                return None
        
        # 转换为numpy数组
        time_history = np.array(time_history)
        x_history = np.array(x_history)
        y_history = np.array(y_history)
        psi_history = np.array(psi_history)
        lateral_errors = np.array(lateral_errors)
        yaw_errors = np.array(yaw_errors)
        steering_angles = np.array(steering_angles)
        velocities = np.array(velocities)
        
        # 计算统计信息
        max_lateral_error = np.max(np.abs(lateral_errors))
        mean_lateral_error = np.mean(np.abs(lateral_errors))
        std_lateral_error = np.std(lateral_errors)
        
        results = {
            'time': time_history,
            'x': x_history,
            'y': y_history,
            'psi': psi_history,
            'lateral_errors': lateral_errors,
            'yaw_errors': yaw_errors,
            'steering_angles': steering_angles,
            'velocities': velocities,
            'max_lateral_error': max_lateral_error,
            'mean_lateral_error': mean_lateral_error,
            'std_lateral_error': std_lateral_error,
            'simulation_time': t
        }
        
        if not no_plot:
            plot_results(road, results)
        
        return results
        
    except Exception as e:
        print(f"仿真过程中出现错误: {e}")
        return None

# =========================
# 结果可视化
# =========================
def plot_results(road, results):
    """
    绘制仿真结果
    """
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle('四环PID控制车道居中控制仿真结果', fontsize=16, fontweight='bold')
    
    # 1. 车辆轨迹
    ax1 = axes[0, 0]
    ax1.plot(road['x'], road['y'], 'k-', linewidth=2, label='道路中心线')
    ax1.plot(road['left_x'], road['left_y'], 'r--', linewidth=1, label='左车道线')
    ax1.plot(road['right_x'], road['right_y'], 'r--', linewidth=1, label='右车道线')
    ax1.plot(results['x'], results['y'], 'b-', linewidth=2, label='车辆轨迹')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_title('车辆轨迹跟踪')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')
    
    # 2. 横向误差
    ax2 = axes[0, 1]
    ax2.plot(results['time'], results['lateral_errors'], 'r-', linewidth=2)
    ax2.axhline(y=0.5, color='g', linestyle='--', label='目标误差±0.5m')
    ax2.axhline(y=-0.5, color='g', linestyle='--')
    ax2.set_xlabel('时间 [s]')
    ax2.set_ylabel('横向误差 [m]')
    ax2.set_title(f'横向误差 (最大: {results["max_lateral_error"]:.3f}m)')
    ax2.legend()
    ax2.grid(True)
    
    # 3. 偏航角误差
    ax3 = axes[0, 2]
    ax3.plot(results['time'], np.rad2deg(results['yaw_errors']), 'b-', linewidth=2)
    ax3.set_xlabel('时间 [s]')
    ax3.set_ylabel('偏航角误差 [度]')
    ax3.set_title('偏航角误差')
    ax3.grid(True)
    
    # 4. 转向角
    ax4 = axes[1, 0]
    ax4.plot(results['time'], np.rad2deg(results['steering_angles']), 'g-', linewidth=2)
    ax4.set_xlabel('时间 [s]')
    ax4.set_ylabel('转向角 [度]')
    ax4.set_title('前轮转向角')
    ax4.grid(True)
    
    # 5. 横向误差分布
    ax5 = axes[1, 1]
    ax5.hist(results['lateral_errors'], bins=50, alpha=0.7, color='orange')
    ax5.axvline(x=0.5, color='r', linestyle='--', label='±0.5m边界')
    ax5.axvline(x=-0.5, color='r', linestyle='--')
    ax5.set_xlabel('横向误差 [m]')
    ax5.set_ylabel('频次')
    ax5.set_title('横向误差分布')
    ax5.legend()
    ax5.grid(True)
    
    # 6. 统计信息
    ax6 = axes[1, 2]
    ax6.axis('off')
    stats_text = f"""
    仿真统计信息:
    
    最大横向误差: {results['max_lateral_error']:.3f} m
    平均横向误差: {results['mean_lateral_error']:.3f} m
    横向误差标准差: {results['std_lateral_error']:.3f} m
    
    仿真时间: {results['simulation_time']:.1f} s
    车辆速度: {results['velocities'][0]:.1f} m/s
    
    控制目标: < 0.5 m
    目标达成: {'✓' if results['max_lateral_error'] < 0.5 else '✗'}
    """
    ax6.text(0.1, 0.5, stats_text, transform=ax6.transAxes, 
             fontsize=12, verticalalignment='center',
             bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.8))
    
    plt.tight_layout()
    plt.show()

# =========================
# 主函数
# =========================
def main():
    """
    主函数：运行完整的LCC仿真
    """
    print("=" * 60)
    print("优化的四环PID控制车道居中控制（LCC）算法")
    print("=" * 60)
    
    # 1. 车辆参数
    params = get_vehicle_params()
    print("\n车辆参数:")
    for key, value in params.items():
        print(f"  {key}: {value}")
    
    # 2. 道路生成
    print("\n生成道路模型...")
    road = generate_smooth_road()
    print(f"道路总长度: {road['s'][-1]:.1f} m")
    print(f"车道宽度: {road['lane_width']} m")
    print(f"弯道半径: {road['curve_radius']} m")
    
    # 3. 初始PID参数
    initial_gains = {
        'e': [0.5, 0.1, 0.05],      # 位置环
        'psi': [1.0, 0.2, 0.1],     # 偏航角环
        'vy': [0.8, 0.15, 0.08],    # 横向速度环
        'r': [1.2, 0.25, 0.12]      # 偏航角速度环
    }
    
    # 4. 参数搜索范围
    search_ranges = [
        (0.1, 2.0),   # Kp范围
        (0.01, 0.5),  # Ki范围
        (0.01, 0.3)   # Kd范围
    ]
    
    # 5. 自动调参
    print("\n开始自动调参...")
    optimal_gains = auto_tune_pid(road, params, initial_gains, search_ranges, params['dt'])
    
    # 6. 运行仿真
    print("\n运行优化后的仿真...")
    results = simulate_lcc(road, params, optimal_gains, params['dt'])
    
    if results is not None:
        print("\n仿真结果:")
        print(f"  最大横向误差: {results['max_lateral_error']:.3f} m")
        print(f"  平均横向误差: {results['mean_lateral_error']:.3f} m")
        print(f"  横向误差标准差: {results['std_lateral_error']:.3f} m")
        print(f"  仿真时间: {results['simulation_time']:.1f} s")
        
        if results['max_lateral_error'] < 0.5:
            print("  ✓ 控制目标达成！横向误差 < 0.5 m")
        else:
            print("  ✗ 控制目标未达成，需要进一步优化")
    
    print("\n仿真完成！")

if __name__ == "__main__":
    main() 