#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LCC简化可视化演示脚本
展示动态动画和性能分析功能

作者：AI Assistant
日期：2024
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
from optimized_lcc_four_loops import *

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

def create_dynamic_animation(road, results, save_path='lcc_animation.gif', fps=20):
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

def create_performance_dashboard(road, results):
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

def create_comprehensive_visualization(road, results):
    """
    创建综合可视化界面
    """
    fig = plt.figure(figsize=(20, 12))
    fig.suptitle('LCC综合可视化界面', fontsize=16, fontweight='bold')
    
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

def main():
    """
    主函数：演示可视化功能
    """
    print("=" * 60)
    print("LCC简化可视化演示")
    print("=" * 60)
    
    # 生成测试数据
    print("1. 生成测试数据...")
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
    print("2. 运行仿真...")
    results = simulate_lcc(road, params, optimal_gains, params['dt'], 
                          max_steps=2000, no_plot=True)
    
    if results is None:
        print("仿真失败！")
        return
    
    print("3. 仿真完成，开始可视化演示...")
    
    # 选择要演示的可视化功能
    print("\n请选择要演示的可视化功能:")
    print("1. 动态动画")
    print("2. 性能仪表板")
    print("3. 综合可视化界面")
    print("4. 所有功能")
    
    choice = input("\n请输入选择 (1-4): ").strip()
    
    if choice == '1':
        print("\n创建动态动画...")
        create_dynamic_animation(road, results, 'lcc_demo_animation.gif')
        
    elif choice == '2':
        print("\n创建性能仪表板...")
        create_performance_dashboard(road, results)
        
    elif choice == '3':
        print("\n创建综合可视化界面...")
        create_comprehensive_visualization(road, results)
        
    elif choice == '4':
        print("\n创建所有可视化界面...")
        
        print("  - 动态动画...")
        create_dynamic_animation(road, results, 'lcc_demo_animation.gif')
        
        print("  - 性能仪表板...")
        create_performance_dashboard(road, results)
        
        print("  - 综合可视化界面...")
        create_comprehensive_visualization(road, results)
        
    else:
        print("无效选择，默认创建性能仪表板...")
        create_performance_dashboard(road, results)
    
    print("\n可视化演示完成！")

if __name__ == "__main__":
    main() 