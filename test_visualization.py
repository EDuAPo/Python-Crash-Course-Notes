#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LCC可视化功能测试脚本
快速验证所有可视化功能是否正常工作

作者：AI Assistant
日期：2024
"""

import numpy as np
import matplotlib.pyplot as plt
import time
from optimized_lcc_four_loops import *
from simple_visualization_demo import *

def test_basic_visualization():
    """
    测试基础可视化功能
    """
    print("=" * 60)
    print("测试基础可视化功能")
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
    start_time = time.time()
    results = simulate_lcc(road, params, optimal_gains, params['dt'], 
                          max_steps=1000, no_plot=True)
    sim_time = time.time() - start_time
    
    if results is None:
        print("❌ 仿真失败！")
        return False
    
    print(f"✅ 仿真成功，耗时: {sim_time:.2f}秒")
    print(f"   - 最大横向误差: {results['max_lateral_error']:.6f} m")
    print(f"   - 平均横向误差: {results['mean_lateral_error']:.6f} m")
    print(f"   - 仿真时间: {results['simulation_time']:.1f} s")
    
    # 测试性能仪表板
    print("3. 测试性能仪表板...")
    try:
        start_time = time.time()
        create_performance_dashboard(road, results)
        viz_time = time.time() - start_time
        print(f"✅ 性能仪表板创建成功，耗时: {viz_time:.2f}秒")
    except Exception as e:
        print(f"❌ 性能仪表板创建失败: {e}")
        return False
    
    # 测试综合可视化界面
    print("4. 测试综合可视化界面...")
    try:
        start_time = time.time()
        create_comprehensive_visualization(road, results)
        viz_time = time.time() - start_time
        print(f"✅ 综合可视化界面创建成功，耗时: {viz_time:.2f}秒")
    except Exception as e:
        print(f"❌ 综合可视化界面创建失败: {e}")
        return False
    
    return True

def test_animation_creation():
    """
    测试动画创建功能（不显示，只保存）
    """
    print("=" * 60)
    print("测试动画创建功能")
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
                          max_steps=500, no_plot=True)
    
    if results is None:
        print("❌ 仿真失败！")
        return False
    
    # 测试动画创建（不显示）
    print("3. 测试动画创建...")
    try:
        # 使用非交互式后端
        plt.switch_backend('Agg')
        
        start_time = time.time()
        anim = create_dynamic_animation(road, results, 'test_animation.gif', fps=10)
        anim_time = time.time() - start_time
        
        print(f"✅ 动画创建成功，耗时: {anim_time:.2f}秒")
        print("   - 动画文件: test_animation.gif")
        print("   - 帧数:", len(results['time']))
        print("   - 帧率: 10 fps")
        
        # 恢复交互式后端
        plt.switch_backend('TkAgg')
        
    except Exception as e:
        print(f"❌ 动画创建失败: {e}")
        return False
    
    return True

def test_parameter_comparison():
    """
    测试参数对比功能
    """
    print("=" * 60)
    print("测试参数对比功能")
    print("=" * 60)
    
    # 生成测试数据
    print("1. 生成测试数据...")
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
        }
    }
    
    # 运行不同参数的仿真
    results_dict = {}
    
    for config_name, gains in pid_configs.items():
        print(f"2. 运行 {config_name} 仿真...")
        results = simulate_lcc(road, params, gains, params['dt'], 
                              max_steps=500, no_plot=True)
        if results is not None:
            results_dict[config_name] = results
            print(f"   ✅ {config_name}: 最大误差 {results['max_lateral_error']:.6f} m")
        else:
            print(f"   ❌ {config_name}: 仿真失败")
    
    if len(results_dict) < 2:
        print("❌ 参数对比测试失败：至少需要2组成功的仿真结果")
        return False
    
    # 创建对比图
    print("3. 创建参数对比图...")
    try:
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('LCC参数对比分析测试', fontsize=16, fontweight='bold')
        
        colors = ['blue', 'red']
        
        # 横向误差对比
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
        
        # 性能指标对比
        ax2 = axes[0, 1]
        config_names = list(results_dict.keys())
        max_errors = [results_dict[name]['max_lateral_error'] for name in config_names]
        
        bars = ax2.bar(config_names, max_errors, color=['green' if e < 0.5 else 'red' for e in max_errors], alpha=0.7)
        ax2.set_ylabel('最大误差 [m]')
        ax2.set_title('性能指标对比')
        ax2.grid(True, axis='y')
        
        # 添加数值标签
        for bar, value in zip(bars, max_errors):
            height = bar.get_height()
            ax2.text(bar.get_x() + bar.get_width()/2., height + 0.001,
                    f'{value:.3f}', ha='center', va='bottom')
        
        # 统计信息
        ax3 = axes[1, :]
        ax3[0].axis('off')
        ax3[1].axis('off')
        
        stats_text = "参数对比测试结果:\n\n"
        for config_name, results in results_dict.items():
            success = "✓" if results['max_lateral_error'] < 0.5 else "✗"
            stats_text += f"{config_name}:\n"
            stats_text += f"  最大误差: {results['max_lateral_error']:.6f} m {success}\n"
            stats_text += f"  平均误差: {results['mean_lateral_error']:.6f} m\n"
            stats_text += f"  标准差: {results['std_lateral_error']:.6f} m\n\n"
        
        ax3[0].text(0.1, 0.5, stats_text, transform=ax3[0].transAxes, 
                   fontsize=11, verticalalignment='center',
                   bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.8))
        
        plt.tight_layout()
        plt.savefig('test_parameter_comparison.png', dpi=150, bbox_inches='tight')
        plt.close()
        
        print("✅ 参数对比图创建成功")
        print("   - 保存文件: test_parameter_comparison.png")
        
    except Exception as e:
        print(f"❌ 参数对比图创建失败: {e}")
        return False
    
    return True

def test_performance_metrics():
    """
    测试性能指标计算
    """
    print("=" * 60)
    print("测试性能指标计算")
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
                          max_steps=1000, no_plot=True)
    
    if results is None:
        print("❌ 仿真失败！")
        return False
    
    # 测试性能指标
    print("3. 计算性能指标...")
    try:
        # 基本指标
        max_error = results['max_lateral_error']
        mean_error = results['mean_lateral_error']
        std_error = results['std_lateral_error']
        
        # 计算其他指标
        settling_time = 0
        for i, error in enumerate(results['lateral_errors']):
            if abs(error) <= 0.5:
                settling_time = results['time'][i]
                break
        
        overshoot = np.max(np.abs(results['lateral_errors']))
        rmse = np.sqrt(np.mean(np.array(results['lateral_errors'])**2))
        
        print("✅ 性能指标计算成功:")
        print(f"   - 最大误差: {max_error:.6f} m")
        print(f"   - 平均误差: {mean_error:.6f} m")
        print(f"   - 误差标准差: {std_error:.6f} m")
        print(f"   - 调节时间: {settling_time:.2f} s")
        print(f"   - 超调量: {overshoot:.6f} m")
        print(f"   - 均方根误差: {rmse:.6f} m")
        
        # 性能评估
        if max_error < 0.5:
            print("   - 控制目标: ✅ 达成")
        else:
            print("   - 控制目标: ❌ 未达成")
        
        if std_error < 0.1:
            print("   - 稳定性: ✅ 优秀")
        elif std_error < 0.2:
            print("   - 稳定性: ⚠️ 良好")
        else:
            print("   - 稳定性: ❌ 需要改进")
        
    except Exception as e:
        print(f"❌ 性能指标计算失败: {e}")
        return False
    
    return True

def main():
    """
    主测试函数
    """
    print("LCC可视化功能测试")
    print("=" * 60)
    
    test_results = []
    
    # 测试1: 基础可视化功能
    print("\n测试1: 基础可视化功能")
    result1 = test_basic_visualization()
    test_results.append(("基础可视化功能", result1))
    
    # 测试2: 动画创建功能
    print("\n测试2: 动画创建功能")
    result2 = test_animation_creation()
    test_results.append(("动画创建功能", result2))
    
    # 测试3: 参数对比功能
    print("\n测试3: 参数对比功能")
    result3 = test_parameter_comparison()
    test_results.append(("参数对比功能", result3))
    
    # 测试4: 性能指标计算
    print("\n测试4: 性能指标计算")
    result4 = test_performance_metrics()
    test_results.append(("性能指标计算", result4))
    
    # 输出测试结果
    print("\n" + "=" * 60)
    print("测试结果总结")
    print("=" * 60)
    
    passed = 0
    total = len(test_results)
    
    for test_name, result in test_results:
        status = "✅ 通过" if result else "❌ 失败"
        print(f"{test_name}: {status}")
        if result:
            passed += 1
    
    print(f"\n总体结果: {passed}/{total} 项测试通过")
    
    if passed == total:
        print("🎉 所有测试通过！可视化模块工作正常。")
    else:
        print("⚠️ 部分测试失败，请检查相关功能。")
    
    print("\n生成的文件:")
    print("- test_animation.gif (如果测试2通过)")
    print("- test_parameter_comparison.png (如果测试3通过)")

if __name__ == "__main__":
    main() 