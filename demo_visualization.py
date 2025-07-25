#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LCC可视化模块演示脚本
展示动态动画、实时监控和性能分析功能

作者：AI Assistant
日期：2024
"""

import numpy as np
import matplotlib.pyplot as plt
from optimized_lcc_four_loops import *
from lcc_visualization_module import LCCVisualizationModule

def demo_basic_visualization():
    """
    基础可视化演示
    """
    print("=" * 60)
    print("LCC可视化模块基础演示")
    print("=" * 60)
    
    # 创建可视化模块
    viz_module = LCCVisualizationModule()
    
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
    print("2. 实时监控界面")
    print("3. 性能仪表板")
    print("4. 3D可视化")
    print("5. 所有功能")
    
    choice = input("\n请输入选择 (1-5): ").strip()
    
    if choice == '1':
        print("\n创建动态动画...")
        viz_module.create_dynamic_animation(road, results, 'lcc_demo_animation.gif')
        
    elif choice == '2':
        print("\n创建实时监控界面...")
        viz_module.create_real_time_monitor(road, results)
        
    elif choice == '3':
        print("\n创建性能仪表板...")
        viz_module.create_performance_dashboard(road, results)
        
    elif choice == '4':
        print("\n创建3D可视化界面...")
        viz_module.create_3d_visualization(road, results)
        
    elif choice == '5':
        print("\n创建所有可视化界面...")
        
        print("  - 动态动画...")
        viz_module.create_dynamic_animation(road, results, 'lcc_demo_animation.gif')
        
        print("  - 实时监控界面...")
        viz_module.create_real_time_monitor(road, results)
        
        print("  - 性能仪表板...")
        viz_module.create_performance_dashboard(road, results)
        
        print("  - 3D可视化界面...")
        viz_module.create_3d_visualization(road, results)
        
    else:
        print("无效选择，默认创建动态动画...")
        viz_module.create_dynamic_animation(road, results, 'lcc_demo_animation.gif')
    
    print("\n可视化演示完成！")

def demo_quick_visualization():
    """
    快速可视化演示（无用户交互）
    """
    print("=" * 60)
    print("LCC快速可视化演示")
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
                          max_steps=1500, no_plot=True)
    
    if results is None:
        print("仿真失败！")
        return
    
    print("仿真完成，创建可视化...")
    
    # 创建性能仪表板（最实用的可视化）
    print("创建性能仪表板...")
    viz_module.create_performance_dashboard(road, results)
    
    print("快速可视化演示完成！")

def demo_parameter_comparison():
    """
    参数对比可视化演示
    """
    print("=" * 60)
    print("LCC参数对比可视化演示")
    print("=" * 60)
    
    # 创建可视化模块
    viz_module = LCCVisualizationModule()
    
    # 生成测试数据
    print("生成测试数据...")
    params = get_vehicle_params()
    road = generate_smooth_road()
    
    # 定义不同的PID参数组合
    pid_configs = {
        '优化参数': {
            'e': [0.2036, 0.3188, 0.1748],
            'psi': [1.5698, 0.0482, 0.0236],
            'vy': [0.1837, 0.4796, 0.2856],
            'r': [1.0627, 0.3497, 0.1980]
        },
        '保守参数': {
            'e': [0.1, 0.05, 0.02],
            'psi': [0.5, 0.02, 0.01],
            'vy': [0.1, 0.1, 0.05],
            'r': [0.5, 0.1, 0.05]
        },
        '激进参数': {
            'e': [1.0, 0.5, 0.3],
            'psi': [2.0, 0.1, 0.05],
            'vy': [1.0, 0.8, 0.5],
            'r': [2.0, 0.5, 0.3]
        }
    }
    
    # 运行不同参数的仿真
    results_dict = {}
    
    for config_name, gains in pid_configs.items():
        print(f"运行 {config_name} 仿真...")
        results = simulate_lcc(road, params, gains, params['dt'], 
                              max_steps=1500, no_plot=True)
        if results is not None:
            results_dict[config_name] = results
            print(f"  - 最大横向误差: {results['max_lateral_error']:.6f} m")
        else:
            print(f"  - 仿真失败")
    
    if not results_dict:
        print("所有仿真都失败了！")
        return
    
    # 创建对比图
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('LCC参数对比分析', fontsize=16, fontweight='bold')
    
    colors = ['blue', 'red', 'green']
    
    # 1. 横向误差对比
    ax1 = axes[0, 0]
    for i, (config_name, results) in enumerate(results_dict.items()):
        ax1.plot(results['time'], results['lateral_errors'], 
                color=colors[i], linewidth=2, label=config_name)
    ax1.axhline(y=0.5, color='g', linestyle='--', alpha=0.7, label='目标误差±0.5m')
    ax1.axhline(y=-0.5, color='g', linestyle='--', alpha=0.7)
    ax1.set_xlabel('时间 [s]')
    ax1.set_ylabel('横向误差 [m]')
    ax1.set_title('横向误差对比')
    ax1.legend()
    ax1.grid(True)
    
    # 2. 转向角对比
    ax2 = axes[0, 1]
    for i, (config_name, results) in enumerate(results_dict.items()):
        ax2.plot(results['time'], np.rad2deg(results['steering_angles']), 
                color=colors[i], linewidth=2, label=config_name)
    ax2.set_xlabel('时间 [s]')
    ax2.set_ylabel('转向角 [度]')
    ax2.set_title('转向角对比')
    ax2.legend()
    ax2.grid(True)
    
    # 3. 性能指标对比
    ax3 = axes[1, 0]
    config_names = list(results_dict.keys())
    max_errors = [results_dict[name]['max_lateral_error'] for name in config_names]
    mean_errors = [results_dict[name]['mean_lateral_error'] for name in config_names]
    
    x = np.arange(len(config_names))
    width = 0.35
    
    bars1 = ax3.bar(x - width/2, max_errors, width, label='最大误差', alpha=0.7)
    bars2 = ax3.bar(x + width/2, mean_errors, width, label='平均误差', alpha=0.7)
    
    ax3.set_xlabel('参数配置')
    ax3.set_ylabel('误差 [m]')
    ax3.set_title('性能指标对比')
    ax3.set_xticks(x)
    ax3.set_xticklabels(config_names)
    ax3.legend()
    ax3.grid(True, axis='y')
    
    # 添加数值标签
    for bars in [bars1, bars2]:
        for bar in bars:
            height = bar.get_height()
            ax3.text(bar.get_x() + bar.get_width()/2., height + 0.001,
                    f'{height:.3f}', ha='center', va='bottom', fontsize=8)
    
    # 4. 统计信息
    ax4 = axes[1, 1]
    ax4.axis('off')
    
    stats_text = "参数对比总结:\n\n"
    for config_name, results in results_dict.items():
        success = "✓" if results['max_lateral_error'] < 0.5 else "✗"
        stats_text += f"{config_name}:\n"
        stats_text += f"  最大误差: {results['max_lateral_error']:.6f} m {success}\n"
        stats_text += f"  平均误差: {results['mean_lateral_error']:.6f} m\n"
        stats_text += f"  标准差: {results['std_lateral_error']:.6f} m\n\n"
    
    ax4.text(0.1, 0.5, stats_text, transform=ax4.transAxes, 
            fontsize=11, verticalalignment='center',
            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.8))
    
    plt.tight_layout()
    plt.show()
    
    print("参数对比可视化演示完成！")

def main():
    """
    主函数
    """
    print("LCC可视化模块演示")
    print("请选择演示模式:")
    print("1. 基础可视化演示（交互式）")
    print("2. 快速可视化演示")
    print("3. 参数对比可视化演示")
    print("4. 退出")
    
    choice = input("\n请输入选择 (1-4): ").strip()
    
    if choice == '1':
        demo_basic_visualization()
    elif choice == '2':
        demo_quick_visualization()
    elif choice == '3':
        demo_parameter_comparison()
    elif choice == '4':
        print("退出演示")
    else:
        print("无效选择，运行快速演示...")
        demo_quick_visualization()

if __name__ == "__main__":
    main() 