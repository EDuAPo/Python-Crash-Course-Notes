#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
优化的车道居中控制（LCC）系统 - 四环PID+前馈控制版本
Optimized Lane Centering Control (LCC) System - Four-Loop PID + Feedforward Control

功能特性:
- 车辆横向动力学模型(自行车模型，4状态变量)
- 四环PID控制器(位置环、偏航环、横向速度环、偏航角速度环)
- 前馈控制补偿道路曲率
- 可调节控制参数模块
- 道路模型: 1000m直道+500m半径弯道+1000m直道，车道宽3.8m
- 仿真跟踪车道中心线验证横向误差<0.5m
- 全英文可视化界面(matplotlib)
- 模块化代码设计

作者：AI Assistant
日期：2024
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, Circle
from scipy.integrate import odeint
import time
import warnings
warnings.filterwarnings('ignore')

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# =========================
# 可调节参数配置类
# =========================
class OptimizedLCCConfig:
    """优化的LCC配置类 - 四环PID+前馈控制"""
    
    def __init__(self):
        # 车辆参数 - 自行车模型
        self.vehicle_params = {
            'm': 1500.0,              # 质量 [kg]
            'Iz': 2500.0,             # 转动惯量 [kg*m^2]
            'lf': 1.2,                # 前轴到质心距离 [m]
            'lr': 1.6,                # 后轴到质心距离 [m]
            'Cf': 80000.0,            # 前轮侧偏刚度 [N/rad]
            'Cr': 80000.0,            # 后轮侧偏刚度 [N/rad]
            'wheelbase': 2.8,         # 轴距 [m]
            'speed': 8.0,             # 车速 [m/s]
            'dt': 0.02,               # 时间步长 [s]
            'max_steer': np.radians(25),  # 最大转向角 [rad]
        }
        
        # 四环PID控制器参数 - 可调节
        self.pid_gains = {
            'e': [0.5, 0.01, 0.1],      # 位置环 [Kp, Ki, Kd]
            'psi': [1.2, 0.01, 0.2],    # 偏航角环 [Kp, Ki, Kd]
            'vy': [0.8, 0.01, 0.1],     # 横向速度环 [Kp, Ki, Kd]
            'r': [1.0, 0.01, 0.1]       # 偏航角速度环 [Kp, Ki, Kd]
        }
        
        # 前馈控制参数
        self.feedforward_gain = 1.5     # 前馈增益
        
        # 道路参数
        self.road_params = {
            'lane_width': 3.8,          # 车道宽度 [m]
            'straight1': 1000.0,        # 第一段直道长度 [m]
            'curve_radius': 500.0,      # 弯道半径 [m]
            'straight2': 1000.0,        # 第二段直道长度 [m]
            'sim_time': 300.0,          # 仿真时间 [s]
        }
        
        # 可视化参数
        self.visualization_params = {
            'animation_interval': 100,   # 动画间隔 [ms]
            'save_animation': True,      # 是否保存动画
            'show_realtime': True,       # 是否实时显示
            'vehicle_size': 6.0,         # 车辆显示大小
            'lane_line_width': 4,        # 车道线宽度
        }

# =========================
# 道路模型生成
# =========================
def generate_road_model(config, ds=0.5):
    """
    生成道路模型：1000m直道 + 500m半径弯道 + 1000m直道
    确保直道与弯道平滑相接
    """
    # 第一段直道
    s_straight1 = np.arange(0, config.road_params['straight1'], ds)
    x1 = s_straight1
    y1 = np.zeros_like(x1)
    psi1 = np.zeros_like(x1)
    kappa1 = np.zeros_like(x1)
    
    # 弯道（左转90度）
    s_curve = np.arange(0, np.pi/2 * config.road_params['curve_radius'], ds)
    angle_curve = s_curve / config.road_params['curve_radius']
    x2 = config.road_params['straight1'] + config.road_params['curve_radius'] * np.sin(angle_curve)
    y2 = config.road_params['curve_radius'] * (1 - np.cos(angle_curve))
    psi2 = angle_curve
    kappa2 = np.ones_like(x2) / config.road_params['curve_radius']
    
    # 第二段直道
    s_straight2 = np.arange(0, config.road_params['straight2'], ds)
    x3 = x2[-1] + s_straight2 * np.cos(psi2[-1])
    y3 = y2[-1] + s_straight2 * np.sin(psi2[-1])
    psi3 = np.ones_like(x3) * psi2[-1]
    kappa3 = np.zeros_like(x3)
    
    # 合并道路段
    x = np.concatenate([x1, x2, x3])
    y = np.concatenate([y1, y2, y3])
    psi = np.concatenate([psi1, psi2, psi3])
    kappa = np.concatenate([kappa1, kappa2, kappa3])
    
    return {
        'x': x,
        'y': y,
        'psi': psi,
        'kappa': kappa,
        's': np.concatenate([s_straight1, s_curve, s_straight2])
    }

# =========================
# 车辆横向动力学模型（自行车模型）
# =========================
def vehicle_lateral_dynamics(state, t, delta, v, config, kappa_ref):
    """
    车辆横向动力学模型 - 自行车模型
    状态变量: [e, psi_err, vy, r]
    e: 横向位置误差
    psi_err: 偏航角误差
    vy: 横向速度
    r: 偏航角速度
    """
    e, psi_err, vy, r = state
    
    # 车辆参数
    m = config.vehicle_params['m']
    Iz = config.vehicle_params['Iz']
    lf = config.vehicle_params['lf']
    lr = config.vehicle_params['lr']
    Cf = config.vehicle_params['Cf']
    Cr = config.vehicle_params['Cr']
    
    # 动力学系数
    a11 = -(2*Cf + 2*Cr) / (m * v)
    a12 = (-2*Cf*lf + 2*Cr*lr) / (m * v) - v
    a21 = (-2*Cf*lf + 2*Cr*lr) / (Iz * v)
    a22 = -(2*Cf*lf**2 + 2*Cr*lr**2) / (Iz * v)
    
    b1 = 2*Cf / m
    b2 = 2*Cf*lf / Iz
    
    # 横向误差和偏航角误差运动学
    de = vy * np.cos(psi_err) + v * np.sin(psi_err)
    dpsi_err = r - kappa_ref * v
    
    # 横向动力学
    dvy = a11*vy + a12*r + b1*delta
    dr = a21*vy + a22*r + b2*delta
    
    return [de, dpsi_err, dvy, dr]

# =========================
# 四环PID + 前馈控制器
# =========================
class FourLoopPIDController:
    """四环PID + 前馈控制器"""
    
    def __init__(self, config):
        self.gains = config.pid_gains
        self.feedforward_gain = config.feedforward_gain
        self.dt = config.vehicle_params['dt']
        self.max_steer = config.vehicle_params['max_steer']
        
        # 积分项
        self.integral = {'e': 0.0, 'psi': 0.0, 'vy': 0.0, 'r': 0.0}
        self.prev_error = {'e': 0.0, 'psi': 0.0, 'vy': 0.0, 'r': 0.0}
        
    def reset(self):
        """重置控制器积分项"""
        for k in self.integral:
            self.integral[k] = 0.0
        for k in self.prev_error:
            self.prev_error[k] = 0.0
    
    def control(self, e, psi_err, vy, r, v, kappa_ref):
        """
        四环PID + 前馈控制
        e: 横向位置误差
        psi_err: 偏航角误差
        vy: 横向速度
        r: 偏航角速度
        v: 纵向速度
        kappa_ref: 参考曲率
        """
        # 四环PID控制
        u_e = self._pid('e', e)
        u_psi = self._pid('psi', psi_err)
        u_vy = self._pid('vy', vy)
        u_r = self._pid('r', r)
        
        # 前馈控制补偿道路曲率
        delta_ff = self.feedforward_gain * kappa_ref
        
        # 总控制输入
        delta = u_e + u_psi + u_vy + u_r + delta_ff
        
        # 限制转向角
        delta = np.clip(delta, -self.max_steer, self.max_steer)
        
        return delta
    
    def _pid(self, name, error):
        """PID控制器"""
        Kp, Ki, Kd = self.gains[name]
        
        # 积分项
        self.integral[name] += error * self.dt
        
        # 微分项
        d_error = (error - self.prev_error[name]) / self.dt
        self.prev_error[name] = error
        
        # PID输出
        return Kp * error + Ki * self.integral[name] + Kd * d_error

# =========================
# 优化的LCC仿真器
# =========================
class OptimizedLCCSimulator:
    """优化的LCC仿真器 - 四环PID+前馈控制"""
    
    def __init__(self, config):
        self.config = config
        self.controller = FourLoopPIDController(config)
        self.road = generate_road_model(config)
        
    def run_simulation(self):
        """运行仿真"""
        print("Starting Four-Loop PID LCC simulation...")
        start_time = time.time()
        
        # 初始化状态: [e, psi_err, vy, r]
        state = [0.0, 0.0, 0.0, 0.0]
        v = self.config.vehicle_params['speed']
        dt = self.config.vehicle_params['dt']
        sim_time = self.config.road_params['sim_time']
        
        # 重置控制器
        self.controller.reset()
        
        # 全局位置初始化
        x, y, psi = 0.0, 0.0, 0.0
        
        # 记录数据
        time_hist = []
        x_hist = []
        y_hist = []
        psi_hist = []
        e_hist = []
        psi_err_hist = []
        delta_hist = []
        ref_x_hist = []
        ref_y_hist = []
        vy_hist = []
        r_hist = []
        
        steps = int(sim_time / dt)
        
        for step in range(steps):
            t = step * dt
            
            # 找到最近的参考点
            dists = (self.road['x'] - x)**2 + (self.road['y'] - y)**2
            idx = np.argmin(dists)
            
            x_ref = self.road['x'][idx]
            y_ref = self.road['y'][idx]
            psi_ref = self.road['psi'][idx]
            kappa_ref = self.road['kappa'][idx]
            
            # 计算横向误差
            dx = x - x_ref
            dy = y - y_ref
            e = -dx * np.sin(psi_ref) + dy * np.cos(psi_ref)
            
            # 计算偏航角误差
            psi_err = psi - psi_ref
            psi_err = np.arctan2(np.sin(psi_err), np.cos(psi_err))
            
            # 四环PID + 前馈控制
            delta = self.controller.control(e, psi_err, state[2], state[3], v, kappa_ref)
            
            # 车辆动力学更新
            state = odeint(vehicle_lateral_dynamics, state, [0, dt], 
                          args=(delta, v, self.config, kappa_ref))[-1]
            
            # 更新全局位置
            x += v * np.cos(psi) * dt
            y += v * np.sin(psi) * dt
            psi += state[3] * dt
            
            # 记录数据
            time_hist.append(t)
            x_hist.append(x)
            y_hist.append(y)
            psi_hist.append(psi)
            e_hist.append(e)
            psi_err_hist.append(psi_err)
            delta_hist.append(delta)
            ref_x_hist.append(x_ref)
            ref_y_hist.append(y_ref)
            vy_hist.append(state[2])
            r_hist.append(state[3])
            
            # 进度显示
            if step % 1000 == 0:
                print(f"Progress: {step/steps*100:.1f}%, Lateral Error: {e:.3f}m")
        
        # 计算统计信息
        max_error = np.max(np.abs(e_hist))
        mean_error = np.mean(np.abs(e_hist))
        rms_error = np.sqrt(np.mean(np.array(e_hist)**2))
        
        simulation_time = time.time() - start_time
        
        print(f"Simulation completed! Time: {simulation_time:.2f}s")
        print(f"Max lateral error: {max_error:.3f}m")
        print(f"Mean lateral error: {mean_error:.3f}m")
        print(f"RMS lateral error: {rms_error:.3f}m")
        
        if max_error < 0.5:
            print("✅ Control target achieved! Lateral error < 0.5m")
        else:
            print("❌ Control target not achieved")
        
        return {
            'time': np.array(time_hist),
            'x': np.array(x_hist),
            'y': np.array(y_hist),
            'psi': np.array(psi_hist),
            'e': np.array(e_hist),
            'psi_err': np.array(psi_err_hist),
            'delta': np.array(delta_hist),
            'ref_x': np.array(ref_x_hist),
            'ref_y': np.array(ref_y_hist),
            'vy': np.array(vy_hist),
            'r': np.array(r_hist),
            'road_x': self.road['x'],
            'road_y': self.road['y'],
            'max_error': max_error,
            'mean_error': mean_error,
            'rms_error': rms_error,
            'simulation_time': simulation_time
        }

# =========================
# 优化的LCC可视化器
# =========================
class OptimizedLCCVisualizer:
    """优化的LCC可视化器 - 全英文界面"""
    
    def __init__(self, config):
        self.config = config
        
    def plot_results(self, results):
        """绘制仿真结果 - 综合分析"""
        fig, axes = plt.subplots(3, 2, figsize=(18, 15))
        fig.suptitle('Four-Loop PID LCC Simulation Results - Comprehensive Analysis', fontsize=16, fontweight='bold')
        
        # 轨迹图 - 全局视图
        ax1 = axes[0, 0]
        
        # 绘制参考轨迹
        ax1.plot(results['road_x'], results['road_y'], 'b--', linewidth=3, label='Reference Centerline', alpha=0.8)
        
        # 绘制车辆轨迹
        ax1.plot(results['x'], results['y'], 'r-', linewidth=3, label='Vehicle Trajectory', alpha=0.8)
        
        # 生成并绘制车道线
        lane_width = self.config.road_params['lane_width']
        left_x = results['road_x'] + lane_width/2 * np.cos(results['road_y'] * 0 + np.pi/2)
        left_y = results['road_y'] + lane_width/2 * np.sin(results['road_y'] * 0 + np.pi/2)
        right_x = results['road_x'] - lane_width/2 * np.cos(results['road_y'] * 0 + np.pi/2)
        right_y = results['road_y'] - lane_width/2 * np.sin(results['road_y'] * 0 + np.pi/2)
        
        ax1.plot(left_x, left_y, 'k-', linewidth=self.config.visualization_params['lane_line_width'], 
                label='Left Lane', alpha=0.8)
        ax1.plot(right_x, right_y, 'k-', linewidth=self.config.visualization_params['lane_line_width'], 
                label='Right Lane', alpha=0.8)
        
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_title('Vehicle Trajectory Tracking - Global View')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 横向误差
        ax2 = axes[0, 1]
        ax2.plot(results['time'], results['e'], 'g-', linewidth=2, label='Lateral Error')
        ax2.axhline(y=0.5, color='r', linestyle='--', alpha=0.7, label='Target Error ±0.5m')
        ax2.axhline(y=-0.5, color='r', linestyle='--', alpha=0.7)
        ax2.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('Lateral Error [m]')
        ax2.set_title('Lateral Error')
        ax2.set_ylim(-1.5, 1.5)
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 转向角
        ax3 = axes[1, 0]
        ax3.plot(results['time'], np.degrees(results['delta']), 'm-', linewidth=2)
        ax3.axhline(y=25, color='r', linestyle='--', alpha=0.7, label='Max Steer ±25°')
        ax3.axhline(y=-25, color='r', linestyle='--', alpha=0.7)
        ax3.set_xlabel('Time [s]')
        ax3.set_ylabel('Steering Angle [deg]')
        ax3.set_title('Steering Angle')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 偏航误差
        ax4 = axes[1, 1]
        ax4.plot(results['time'], np.degrees(results['psi_err']), 'c-', linewidth=2)
        ax4.set_xlabel('Time [s]')
        ax4.set_ylabel('Yaw Error [deg]')
        ax4.set_title('Yaw Error')
        ax4.grid(True, alpha=0.3)
        
        # 横向速度
        ax5 = axes[2, 0]
        ax5.plot(results['time'], results['vy'], 'orange', linewidth=2, label='Lateral Velocity')
        ax5.axhline(y=0, color='b', linestyle='--', alpha=0.7, label='Target vy = 0')
        ax5.set_xlabel('Time [s]')
        ax5.set_ylabel('Lateral Velocity [m/s]')
        ax5.set_title('Lateral Velocity')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        
        # 偏航角速度
        ax6 = axes[2, 1]
        ax6.plot(results['time'], np.degrees(results['r']), 'purple', linewidth=2, label='Yaw Rate')
        ax6.set_xlabel('Time [s]')
        ax6.set_ylabel('Yaw Rate [deg/s]')
        ax6.set_title('Yaw Rate')
        ax6.legend()
        ax6.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
        
        # 性能统计图
        self.plot_performance_stats(results)
        
    def plot_performance_stats(self, results):
        """绘制性能统计图"""
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Four-Loop PID LCC Control Performance Analysis', fontsize=16, fontweight='bold')
        
        # 误差分布直方图
        ax1 = axes[0, 0]
        ax1.hist(results['e'], bins=30, alpha=0.7, color='skyblue', edgecolor='black')
        ax1.axvline(x=0.5, color='r', linestyle='--', alpha=0.8, label='Target ±0.5m')
        ax1.axvline(x=-0.5, color='r', linestyle='--', alpha=0.8)
        ax1.axvline(x=np.mean(results['e']), color='g', linestyle='-', alpha=0.8, 
                   label=f'Mean {np.mean(results["e"]):.3f}m')
        ax1.set_xlabel('Lateral Error [m]')
        ax1.set_ylabel('Frequency')
        ax1.set_title('Lateral Error Distribution')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # 转向角分布
        ax2 = axes[0, 1]
        steering_deg = np.degrees(results['delta'])
        ax2.hist(steering_deg, bins=30, alpha=0.7, color='lightcoral', edgecolor='black')
        ax2.axvline(x=25, color='r', linestyle='--', alpha=0.8, label='Max ±25°')
        ax2.axvline(x=-25, color='r', linestyle='--', alpha=0.8)
        ax2.set_xlabel('Steering Angle [deg]')
        ax2.set_ylabel('Frequency')
        ax2.set_title('Steering Angle Distribution')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 误差随时间变化趋势
        ax3 = axes[1, 0]
        # 使用移动平均平滑误差曲线
        window_size = 50
        if len(results['e']) > window_size:
            error_smooth = np.convolve(results['e'], np.ones(window_size)/window_size, mode='valid')
            time_smooth = results['time'][window_size-1:]
            ax3.plot(time_smooth, error_smooth, 'b-', linewidth=2, label='Smoothed Error')
        ax3.plot(results['time'], results['e'], 'g-', alpha=0.3, linewidth=1, label='Raw Error')
        ax3.axhline(y=0.5, color='r', linestyle='--', alpha=0.7, label='Target ±0.5m')
        ax3.axhline(y=-0.5, color='r', linestyle='--', alpha=0.7)
        ax3.set_xlabel('Time [s]')
        ax3.set_ylabel('Lateral Error [m]')
        ax3.set_title('Error Trend Over Time')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 性能指标总结
        ax4 = axes[1, 1]
        ax4.axis('off')
        
        # 计算性能指标
        max_error = np.max(np.abs(results['e']))
        mean_error = np.mean(np.abs(results['e']))
        rms_error = np.sqrt(np.mean(results['e']**2))
        std_error = np.std(results['e'])
        max_steering = np.max(np.abs(np.degrees(results['delta'])))
        
        # 创建性能总结文本
        performance_text = f"""
Four-Loop PID LCC Performance Summary:
        
Max Lateral Error: {max_error:.3f} m
Mean Lateral Error: {mean_error:.3f} m
RMS Lateral Error: {rms_error:.3f} m
Error Std Dev: {std_error:.3f} m
Max Steering Angle: {max_steering:.1f}°

Control Target: Lateral Error < 0.5m
Target Achieved: {'✅ Yes' if max_error < 0.5 else '❌ No'}

Simulation Time: {results['time'][-1]:.1f} s
Data Points: {len(results['time'])}

Four-Loop PID Gains:
Position Loop: {self.config.pid_gains['e']}
Yaw Loop: {self.config.pid_gains['psi']}
Lateral Velocity Loop: {self.config.pid_gains['vy']}
Yaw Rate Loop: {self.config.pid_gains['r']}
Feedforward Gain: {self.config.feedforward_gain}
        """
        
        ax4.text(0.1, 0.9, performance_text, transform=ax4.transAxes, 
                fontsize=10, verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.8))
        
        plt.tight_layout()
        plt.show()
        
    def create_animation(self, results):
        """创建动态动画"""
        print("Generating Four-Loop PID LCC animation...")
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 8))
        fig.suptitle('Four-Loop PID LCC Dynamic Simulation', fontsize=16, fontweight='bold')
        
        # 设置轨迹图
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_title('Vehicle Trajectory Tracking - Global View')
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 绘制参考轨迹
        ax1.plot(results['road_x'], results['road_y'], 'b--', linewidth=3, label='Reference Centerline', alpha=0.8)
        
        # 设置误差图
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('Lateral Error [m]')
        ax2.set_title('Real-time Lateral Error')
        ax2.grid(True, alpha=0.3)
        ax2.axhline(y=0.5, color='g', linestyle='--', alpha=0.7, label='Target Error ±0.5m')
        ax2.axhline(y=-0.5, color='g', linestyle='--', alpha=0.7)
        ax2.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        ax2.set_ylim(-1.5, 1.5)
        
        # 初始化车辆图形
        vehicle_size = self.config.visualization_params['vehicle_size']
        vehicle_rect = Rectangle((0, 0), vehicle_size, vehicle_size, 
                               angle=0, alpha=0.8, color='red', label='Vehicle')
        ax1.add_patch(vehicle_rect)
        
        # 初始化轨迹线
        trajectory_line, = ax1.plot([], [], 'r-', linewidth=3, label='Vehicle Trajectory')
        error_line, = ax2.plot([], [], 'g-', linewidth=2, label='Lateral Error')
        current_error_point, = ax2.plot([], [], 'ro', markersize=10)
        
        # 设置坐标轴范围
        margin = 50
        ax1.set_xlim(min(results['road_x']) - margin, max(results['road_x']) + margin)
        ax1.set_ylim(min(results['road_y']) - margin, max(results['road_y']) + margin)
        ax2.set_xlim(0, results['time'][-1])
        
        ax1.legend()
        ax2.legend()
        
        def animate(frame):
            # 更新车辆位置
            x = results['x'][frame]
            y = results['y'][frame]
            psi = results['psi'][frame]
            
            vehicle_rect.set_xy((x - vehicle_size/2, y - vehicle_size/2))
            vehicle_rect.angle = np.degrees(psi)
            
            # 更新轨迹
            trajectory_line.set_data(results['x'][:frame+1], results['y'][:frame+1])
            
            # 更新误差
            error_line.set_data(results['time'][:frame+1], results['e'][:frame+1])
            current_error_point.set_data([results['time'][frame]], [results['e'][frame]])
            
            # 更新标题
            current_error = results['e'][frame]
            current_steering = np.degrees(results['delta'][frame])
            ax1.set_title(f'Vehicle Trajectory - Lateral Error: {current_error:.3f}m, Steering: {current_steering:.1f}°')
            
            return vehicle_rect, trajectory_line, error_line, current_error_point
        
        # 创建动画
        anim = animation.FuncAnimation(
            fig, animate, frames=len(results['time']), 
            interval=self.config.visualization_params['animation_interval'],
            blit=True, repeat=True
        )
        
        if self.config.visualization_params['save_animation']:
            print("Saving animation...")
            anim.save('four_loop_pid_lcc_animation.gif', writer='pillow', fps=20)
            print("Animation saved as four_loop_pid_lcc_animation.gif")
        
        plt.show()
        return anim

# =========================
# 主函数
# =========================
def main():
    """主函数"""
    print("=" * 60)
    print("Four-Loop PID + Feedforward LCC System")
    print("=" * 60)
    
    # 创建配置
    config = OptimizedLCCConfig()
    
    print("\n=== Configuration Parameters ===")
    print(f"Vehicle Speed: {config.vehicle_params['speed']} m/s")
    print(f"Simulation Time: {config.road_params['sim_time']} s")
    print(f"Time Step: {config.vehicle_params['dt']} s")
    print(f"Road: {config.road_params['straight1']}m straight + {config.road_params['curve_radius']}m radius curve + {config.road_params['straight2']}m straight")
    print(f"Lane Width: {config.road_params['lane_width']} m")
    
    print("\n=== Four-Loop PID Controller Parameters ===")
    print("Position Loop (e):", config.pid_gains['e'])
    print("Yaw Loop (psi):", config.pid_gains['psi'])
    print("Lateral Velocity Loop (vy):", config.pid_gains['vy'])
    print("Yaw Rate Loop (r):", config.pid_gains['r'])
    print(f"Feedforward Gain: {config.feedforward_gain}")
    
    # 创建仿真器
    simulator = OptimizedLCCSimulator(config)
    
    # 运行仿真
    results = simulator.run_simulation()
    
    if results is None:
        print("Simulation failed!")
        return
    
    # 创建可视化器
    visualizer = OptimizedLCCVisualizer(config)
    
    # 绘制结果
    print("\n=== Plotting Simulation Results ===")
    visualizer.plot_results(results)
    
    # 创建动画
    print("\n=== Creating Dynamic Animation ===")
    anim = visualizer.create_animation(results)
    
    print("\nSimulation completed!")
    print("=" * 60)

if __name__ == "__main__":
    main() 