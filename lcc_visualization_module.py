#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
车道居中控制（LCC）可视化界面模块
包含动态动画、实时监控和交互式参数调整功能

作者：AI Assistant
日期：2024
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button, RadioButtons
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D
import threading
import time
from optimized_lcc_four_loops import *

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

class LCCVisualizationModule:
    """
    LCC可视化模块主类
    """
    def __init__(self):
        self.road = None
        self.results = None
        self.params = None
        self.pid_gains = None
        self.animation_running = False
        
    def create_dynamic_animation(self, road, results, save_path='lcc_animation.gif', fps=20):
        """
        创建动态动画，展示车辆轨迹跟踪过程
        """
        print("正在生成动态动画...")
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
        fig.suptitle('车道居中控制动态仿真', fontsize=16, fontweight='bold')
        
        # 设置轨迹图
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_title('车辆轨迹跟踪')
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 绘制道路
        ax1.plot(road['x'], road['y'], 'k-', linewidth=3, label='道路中心线', alpha=0.7)
        ax1.plot(road['left_x'], road['left_y'], 'r--', linewidth=2, label='左车道线', alpha=0.5)
        ax1.plot(road['right_x'], road['right_y'], 'r--', linewidth=2, label='右车道线', alpha=0.5)
        
        # 设置误差图
        ax2.set_xlabel('时间 [s]')
        ax2.set_ylabel('横向误差 [m]')
        ax2.set_title('实时横向误差')
        ax2.grid(True, alpha=0.3)
        ax2.axhline(y=0.5, color='g', linestyle='--', alpha=0.7, label='目标误差±0.5m')
        ax2.axhline(y=-0.5, color='g', linestyle='--', alpha=0.7)
        ax2.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        
        # 初始化车辆图形
        vehicle_size = 2.0
        vehicle_rect = patches.Rectangle((0, 0), vehicle_size, vehicle_size, 
                                       angle=0, alpha=0.8, color='blue', 
                                       label='车辆')
        ax1.add_patch(vehicle_rect)
        
        # 初始化误差线
        error_line, = ax2.plot([], [], 'r-', linewidth=2, label='横向误差')
        current_error_point, = ax2.plot([], [], 'ro', markersize=8)
        
        # 添加文本显示
        time_text = ax1.text(0.02, 0.98, '', transform=ax1.transAxes, 
                           fontsize=12, verticalalignment='top',
                           bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.8))
        error_text = ax1.text(0.02, 0.92, '', transform=ax1.transAxes, 
                            fontsize=12, verticalalignment='top',
                            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.8))
        
        # 设置图例
        ax1.legend(loc='upper right')
        ax2.legend(loc='upper right')
        
        def animate(frame):
            if frame >= len(results['time']):
                return vehicle_rect, error_line, current_error_point, time_text, error_text
            
            # 更新车辆位置和方向
            x = results['x'][frame]
            y = results['y'][frame]
            psi = results['psi'][frame]
            
            # 计算车辆四个角点
            cos_psi = np.cos(psi)
            sin_psi = np.sin(psi)
            half_size = vehicle_size / 2
            
            corners = np.array([
                [-half_size, -half_size],
                [half_size, -half_size],
                [half_size, half_size],
                [-half_size, half_size]
            ])
            
            # 旋转和平移
            rotation_matrix = np.array([[cos_psi, -sin_psi], [sin_psi, cos_psi]])
            rotated_corners = corners @ rotation_matrix.T
            vehicle_corners = rotated_corners + np.array([x, y])
            
            # 更新车辆矩形
            vehicle_rect.set_xy(vehicle_corners[0])
            vehicle_rect.set_width(vehicle_size)
            vehicle_rect.set_height(vehicle_size)
            vehicle_rect.set_angle(np.rad2deg(psi))
            
            # 更新误差图
            time_data = results['time'][:frame+1]
            error_data = results['lateral_errors'][:frame+1]
            error_line.set_data(time_data, error_data)
            current_error_point.set_data([results['time'][frame]], [results['lateral_errors'][frame]])
            
            # 更新文本显示
            time_text.set_text(f'时间: {results["time"][frame]:.1f} s')
            error_text.set_text(f'横向误差: {results["lateral_errors"][frame]:.3f} m')
            
            # 动态调整坐标轴
            if frame > 0:
                ax2.set_xlim(0, max(results['time'][frame], 10))
                error_range = max(abs(min(error_data)), abs(max(error_data)), 0.6)
                ax2.set_ylim(-error_range, error_range)
            
            return vehicle_rect, error_line, current_error_point, time_text, error_text
        
        # 创建动画
        anim = animation.FuncAnimation(fig, animate, frames=len(results['time']), 
                                     interval=1000/fps, blit=True, repeat=True)
        
        # 保存动画
        if save_path:
            print(f"正在保存动画到: {save_path}")
            anim.save(save_path, writer='pillow', fps=fps)
            print("动画保存完成！")
        
        plt.tight_layout()
        plt.show()
        
        return anim

    def create_real_time_monitor(self, road, results):
        """
        创建实时监控界面
        """
        fig = plt.figure(figsize=(20, 12))
        fig.suptitle('LCC实时监控界面', fontsize=16, fontweight='bold')
        
        # 创建子图
        gs = fig.add_gridspec(3, 4, hspace=0.3, wspace=0.3)
        
        # 1. 车辆轨迹图
        ax1 = fig.add_subplot(gs[0, :2])
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
        ax2 = fig.add_subplot(gs[0, 2:])
        ax2.plot(results['time'], results['lateral_errors'], 'r-', linewidth=2)
        ax2.axhline(y=0.5, color='g', linestyle='--', label='目标误差±0.5m')
        ax2.axhline(y=-0.5, color='g', linestyle='--')
        ax2.set_xlabel('时间 [s]')
        ax2.set_ylabel('横向误差 [m]')
        ax2.set_title(f'横向误差 (最大: {results["max_lateral_error"]:.3f}m)')
        ax2.legend()
        ax2.grid(True)
        
        # 3. 偏航角误差
        ax3 = fig.add_subplot(gs[1, :2])
        ax3.plot(results['time'], np.rad2deg(results['yaw_errors']), 'b-', linewidth=2)
        ax3.set_xlabel('时间 [s]')
        ax3.set_ylabel('偏航角误差 [度]')
        ax3.set_title('偏航角误差')
        ax3.grid(True)
        
        # 4. 转向角
        ax4 = fig.add_subplot(gs[1, 2:])
        ax4.plot(results['time'], np.rad2deg(results['steering_angles']), 'g-', linewidth=2)
        ax4.set_xlabel('时间 [s]')
        ax4.set_ylabel('转向角 [度]')
        ax4.set_title('前轮转向角')
        ax4.grid(True)
        
        # 5. 横向误差分布
        ax5 = fig.add_subplot(gs[2, :2])
        ax5.hist(results['lateral_errors'], bins=50, alpha=0.7, color='orange', edgecolor='black')
        ax5.axvline(x=0.5, color='r', linestyle='--', label='±0.5m边界')
        ax5.axvline(x=-0.5, color='r', linestyle='--')
        ax5.set_xlabel('横向误差 [m]')
        ax5.set_ylabel('频次')
        ax5.set_title('横向误差分布')
        ax5.legend()
        ax5.grid(True)
        
        # 6. 统计信息
        ax6 = fig.add_subplot(gs[2, 2:])
        ax6.axis('off')
        stats_text = f"""
        仿真统计信息:
        
        最大横向误差: {results['max_lateral_error']:.6f} m
        平均横向误差: {results['mean_lateral_error']:.6f} m
        横向误差标准差: {results['std_lateral_error']:.6f} m
        
        仿真时间: {results['simulation_time']:.1f} s
        车辆速度: {results['velocities'][0]:.1f} m/s
        
        控制目标: < 0.5 m
        目标达成: {'✓' if results['max_lateral_error'] < 0.5 else '✗'}
        
        道路总长度: {road['s'][-1]:.1f} m
        车道宽度: {road['lane_width']} m
        弯道半径: {road['curve_radius']} m
        """
        ax6.text(0.1, 0.5, stats_text, transform=ax6.transAxes, 
                fontsize=11, verticalalignment='center',
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.8))
        
        plt.tight_layout()
        plt.show()

    def create_interactive_parameter_tuner(self, road, params):
        """
        创建交互式参数调整界面
        """
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
        fig.suptitle('LCC参数交互式调整界面', fontsize=16, fontweight='bold')
        
        # 初始化PID参数
        initial_gains = {
            'e': [0.2036, 0.3188, 0.1748],
            'psi': [1.5698, 0.0482, 0.0236],
            'vy': [0.1837, 0.4796, 0.2856],
            'r': [1.0627, 0.3497, 0.1980]
        }
        
        # 创建滑块
        sliders = {}
        slider_positions = {
            'e_kp': [0.1, 0.05, 0.8, 0.02],
            'e_ki': [0.1, 0.25, 0.8, 0.02],
            'e_kd': [0.1, 0.45, 0.8, 0.02],
            'psi_kp': [0.1, 0.65, 0.8, 0.02],
            'psi_ki': [0.1, 0.85, 0.8, 0.02],
            'psi_kd': [0.1, 1.05, 0.8, 0.02],
            'vy_kp': [0.1, 1.25, 0.8, 0.02],
            'vy_ki': [0.1, 1.45, 0.8, 0.02],
            'vy_kd': [0.1, 1.65, 0.8, 0.02],
            'r_kp': [0.1, 1.85, 0.8, 0.02],
            'r_ki': [0.1, 2.05, 0.8, 0.02],
            'r_kd': [0.1, 2.25, 0.8, 0.02]
        }
        
        # 创建滑块
        for param_name, (left, bottom, width, height) in slider_positions.items():
            ax_slider = plt.axes([left, bottom, width, height])
            
            # 确定参数范围
            if 'kp' in param_name:
                valmin, valmax = 0.1, 3.0
                valinit = initial_gains[param_name[:2]][0]
            elif 'ki' in param_name:
                valmin, valmax = 0.01, 1.0
                valinit = initial_gains[param_name[:2]][1]
            else:  # kd
                valmin, valmax = 0.01, 0.5
                valinit = initial_gains[param_name[:2]][2]
            
            slider = Slider(ax_slider, param_name, valmin, valmax, valinit=valinit)
            sliders[param_name] = slider
        
        # 创建按钮
        ax_reset = plt.axes([0.8, 0.9, 0.1, 0.05])
        ax_run = plt.axes([0.8, 0.8, 0.1, 0.05])
        ax_optimize = plt.axes([0.8, 0.7, 0.1, 0.05])
        
        button_reset = Button(ax_reset, '重置')
        button_run = Button(ax_run, '运行')
        button_optimize = Button(ax_optimize, '自动优化')
        
        # 创建结果显示区域
        ax_results = plt.axes([0.8, 0.1, 0.18, 0.5])
        ax_results.axis('off')
        
        def update_results_text(text):
            ax_results.clear()
            ax_results.axis('off')
            ax_results.text(0.1, 0.5, text, transform=ax_results.transAxes, 
                          fontsize=10, verticalalignment='center',
                          bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.8))
            plt.draw()
        
        def get_current_gains():
            """获取当前滑块值"""
            gains = {
                'e': [sliders['e_kp'].val, sliders['e_ki'].val, sliders['e_kd'].val],
                'psi': [sliders['psi_kp'].val, sliders['psi_ki'].val, sliders['psi_kd'].val],
                'vy': [sliders['vy_kp'].val, sliders['vy_ki'].val, sliders['vy_kd'].val],
                'r': [sliders['r_kp'].val, sliders['r_ki'].val, sliders['r_kd'].val]
            }
            return gains
        
        def run_simulation(event):
            """运行仿真"""
            update_results_text("正在运行仿真...")
            plt.draw()
            
            try:
                gains = get_current_gains()
                results = simulate_lcc(road, params, gains, params['dt'], 
                                     max_steps=2000, no_plot=True)
                
                if results is not None:
                    text = f"""
                    仿真结果:
                    
                    最大横向误差: {results['max_lateral_error']:.6f} m
                    平均横向误差: {results['mean_lateral_error']:.6f} m
                    横向误差标准差: {results['std_lateral_error']:.6f} m
                    
                    控制目标: < 0.5 m
                    目标达成: {'✓' if results['max_lateral_error'] < 0.5 else '✗'}
                    """
                else:
                    text = "仿真失败！"
                
                update_results_text(text)
                
            except Exception as e:
                update_results_text(f"仿真错误: {str(e)}")
        
        def reset_parameters(event):
            """重置参数"""
            for param_name, slider in sliders.items():
                if 'kp' in param_name:
                    slider.set_val(initial_gains[param_name[:2]][0])
                elif 'ki' in param_name:
                    slider.set_val(initial_gains[param_name[:2]][1])
                else:
                    slider.set_val(initial_gains[param_name[:2]][2])
            update_results_text("参数已重置")
        
        def auto_optimize(event):
            """自动优化参数"""
            update_results_text("正在自动优化参数...")
            plt.draw()
            
            try:
                search_ranges = [(0.1, 3.0), (0.01, 1.0), (0.01, 0.5)]
                optimal_gains = auto_tune_pid(road, params, initial_gains, 
                                            search_ranges, params['dt'], 
                                            max_iterations=50, verbose=False)
                
                # 更新滑块值
                for param_name, slider in sliders.items():
                    if 'kp' in param_name:
                        slider.set_val(optimal_gains[param_name[:2]][0])
                    elif 'ki' in param_name:
                        slider.set_val(optimal_gains[param_name[:2]][1])
                    else:
                        slider.set_val(optimal_gains[param_name[:2]][2])
                
                update_results_text("参数优化完成！")
                
            except Exception as e:
                update_results_text(f"优化错误: {str(e)}")
        
        # 绑定事件
        button_run.on_clicked(run_simulation)
        button_reset.on_clicked(reset_parameters)
        button_optimize.on_clicked(auto_optimize)
        
        # 初始化结果显示
        update_results_text("请点击'运行'按钮开始仿真")
        
        plt.tight_layout()
        plt.show()

    def create_3d_visualization(self, road, results):
        """
        创建3D可视化界面
        """
        fig = plt.figure(figsize=(16, 12))
        fig.suptitle('LCC 3D可视化界面', fontsize=16, fontweight='bold')
        
        # 创建3D子图
        ax1 = fig.add_subplot(221, projection='3d')
        ax2 = fig.add_subplot(222, projection='3d')
        ax3 = fig.add_subplot(223)
        ax4 = fig.add_subplot(224)
        
        # 1. 3D轨迹图
        ax1.plot3D(road['x'], road['y'], road['s'], 'k-', linewidth=2, label='道路中心线')
        ax1.plot3D(results['x'], results['y'], results['time'], 'b-', linewidth=2, label='车辆轨迹')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_zlabel('时间 [s]')
        ax1.set_title('3D轨迹跟踪')
        ax1.legend()
        
        # 2. 3D误差图
        ax2.plot3D(results['time'], results['lateral_errors'], results['yaw_errors'], 
                  'r-', linewidth=2, label='误差轨迹')
        ax2.set_xlabel('时间 [s]')
        ax2.set_ylabel('横向误差 [m]')
        ax2.set_zlabel('偏航角误差 [rad]')
        ax2.set_title('3D误差空间')
        ax2.legend()
        
        # 3. 误差相图
        ax3.plot(results['lateral_errors'], results['yaw_errors'], 'b-', linewidth=1, alpha=0.7)
        ax3.scatter(results['lateral_errors'][0], results['yaw_errors'][0], 
                   c='green', s=100, label='起点', zorder=5)
        ax3.scatter(results['lateral_errors'][-1], results['yaw_errors'][-1], 
                   c='red', s=100, label='终点', zorder=5)
        ax3.set_xlabel('横向误差 [m]')
        ax3.set_ylabel('偏航角误差 [rad]')
        ax3.set_title('误差相图')
        ax3.legend()
        ax3.grid(True)
        
        # 4. 控制输入相图
        ax4.plot(results['steering_angles'][:-1], np.diff(results['steering_angles']), 
                'g-', linewidth=1, alpha=0.7)
        ax4.set_xlabel('转向角 [rad]')
        ax4.set_ylabel('转向角变化率 [rad/s]')
        ax4.set_title('控制输入相图')
        ax4.grid(True)
        
        plt.tight_layout()
        plt.show()

    def create_performance_dashboard(self, road, results):
        """
        创建性能仪表板
        """
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        fig.suptitle('LCC性能仪表板', fontsize=16, fontweight='bold')
        
        # 1. 控制性能指标
        ax1 = axes[0, 0]
        metrics = ['最大误差', '平均误差', '误差标准差', '控制目标']
        values = [results['max_lateral_error'], results['mean_lateral_error'], 
                 results['std_lateral_error'], 0.5]
        colors = ['red' if v > 0.5 else 'green' for v in values[:3]] + ['blue']
        
        bars = ax1.bar(metrics, values, color=colors, alpha=0.7)
        ax1.set_ylabel('误差 [m]')
        ax1.set_title('控制性能指标')
        ax1.grid(True, axis='y')
        
        # 添加数值标签
        for bar, value in zip(bars, values):
            height = bar.get_height()
            ax1.text(bar.get_x() + bar.get_width()/2., height + 0.01,
                    f'{value:.3f}', ha='center', va='bottom')
        
        # 2. 误差分布
        ax2 = axes[0, 1]
        ax2.hist(results['lateral_errors'], bins=30, alpha=0.7, color='orange', edgecolor='black')
        ax2.axvline(x=0.5, color='r', linestyle='--', label='±0.5m边界')
        ax2.axvline(x=-0.5, color='r', linestyle='--')
        ax2.set_xlabel('横向误差 [m]')
        ax2.set_ylabel('频次')
        ax2.set_title('误差分布')
        ax2.legend()
        ax2.grid(True)
        
        # 3. 控制输入统计
        ax3 = axes[0, 2]
        steering_stats = [
            np.max(np.abs(results['steering_angles'])),
            np.mean(np.abs(results['steering_angles'])),
            np.std(results['steering_angles'])
        ]
        steering_labels = ['最大转向角', '平均转向角', '转向角标准差']
        ax3.bar(steering_labels, steering_stats, color='purple', alpha=0.7)
        ax3.set_ylabel('转向角 [rad]')
        ax3.set_title('控制输入统计')
        ax3.grid(True, axis='y')
        
        # 4. 时间序列分析
        ax4 = axes[1, 0]
        ax4.plot(results['time'], results['lateral_errors'], 'r-', linewidth=2, label='横向误差')
        ax4.plot(results['time'], np.rad2deg(results['yaw_errors'])/10, 'b-', linewidth=2, label='偏航角误差/10')
        ax4.axhline(y=0.5, color='g', linestyle='--', alpha=0.7)
        ax4.axhline(y=-0.5, color='g', linestyle='--', alpha=0.7)
        ax4.set_xlabel('时间 [s]')
        ax4.set_ylabel('误差 [m]')
        ax4.set_title('时间序列分析')
        ax4.legend()
        ax4.grid(True)
        
        # 5. 频谱分析
        ax5 = axes[1, 1]
        # 计算FFT
        fft_error = np.fft.fft(results['lateral_errors'])
        freqs = np.fft.fftfreq(len(results['lateral_errors']), results['time'][1] - results['time'][0])
        positive_freqs = freqs[:len(freqs)//2]
        positive_fft = np.abs(fft_error[:len(freqs)//2])
        
        ax5.plot(positive_freqs, positive_fft, 'g-', linewidth=2)
        ax5.set_xlabel('频率 [Hz]')
        ax5.set_ylabel('幅值')
        ax5.set_title('误差频谱分析')
        ax5.grid(True)
        
        # 6. 性能总结
        ax6 = axes[1, 2]
        ax6.axis('off')
        
        # 计算性能指标
        overshoot = np.max(np.abs(results['lateral_errors']))
        settling_time = 0
        for i, error in enumerate(results['lateral_errors']):
            if abs(error) <= 0.5:
                settling_time = results['time'][i]
                break
        
        performance_text = f"""
        性能总结:
        
        ✓ 控制精度: {results['max_lateral_error']:.6f} m
        ✓ 稳定性: {'优秀' if results['std_lateral_error'] < 0.1 else '良好'}
        ✓ 响应速度: {settling_time:.2f} s
        
        仿真参数:
        • 车速: {results['velocities'][0]:.1f} m/s
        • 仿真时间: {results['simulation_time']:.1f} s
        • 道路长度: {road['s'][-1]:.1f} m
        
        控制目标达成: {'✓' if results['max_lateral_error'] < 0.5 else '✗'}
        """
        
        ax6.text(0.1, 0.5, performance_text, transform=ax6.transAxes, 
                fontsize=11, verticalalignment='center',
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.8))
        
        plt.tight_layout()
        plt.show()

def main():
    """
    主函数：演示所有可视化功能
    """
    print("=" * 60)
    print("LCC可视化界面模块演示")
    print("=" * 60)
    
    # 创建可视化模块
    viz_module = LCCVisualizationModule()
    
    # 生成测试数据
    print("生成测试数据...")
    params = get_vehicle_params()
    road = generate_smooth_road()
    
    # 使用优化后的PID参数
    optimal_gains = {
        'e': [0.2036, 0.3188, 0.1748],
        'psi': [1.5698, 0.0482, 0.0236],
        'vy': [0.1837, 0.4796, 0.2856],
        'r': [1.0627, 0.3497, 0.1980]
    }
    
    # 运行仿真
    print("运行仿真...")
    results = simulate_lcc(road, params, optimal_gains, params['dt'], 
                          max_steps=3000, no_plot=True)
    
    if results is None:
        print("仿真失败！")
        return
    
    print("仿真完成，开始可视化...")
    
    # 1. 动态动画
    print("\n1. 创建动态动画...")
    viz_module.create_dynamic_animation(road, results, 'lcc_dynamic_animation.gif')
    
    # 2. 实时监控界面
    print("\n2. 创建实时监控界面...")
    viz_module.create_real_time_monitor(road, results)
    
    # 3. 交互式参数调整
    print("\n3. 创建交互式参数调整界面...")
    viz_module.create_interactive_parameter_tuner(road, params)
    
    # 4. 3D可视化
    print("\n4. 创建3D可视化界面...")
    viz_module.create_3d_visualization(road, results)
    
    # 5. 性能仪表板
    print("\n5. 创建性能仪表板...")
    viz_module.create_performance_dashboard(road, results)
    
    print("\n所有可视化界面创建完成！")

if __name__ == "__main__":
    main() 