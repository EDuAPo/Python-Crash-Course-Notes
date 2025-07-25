import numpy as np
import matplotlib.pyplot as plt

# 车道参数
LANE_WIDTH = 3.8  # 车道宽度 [m]

def reference_path_highway(t, v, path_type='highway_curve'):
    """
    高速公路轨迹：1000米直道 + 半径500米弯道
    修正版本：确保轨迹连续性
    """
    if path_type == 'highway_curve':
        # 1000米直道段
        straight_length = 1000.0
        straight_time = straight_length / v
        
        if t <= straight_time:
            # 直道段：沿x轴正方向
            x = v * t
            y = 0.0
            psi_ref = 0.0
            curvature = 0.0
        else:
            # 半径500米弯道段：从直道末端开始，向右转弯
            R = 500.0
            arc_length = v * (t - straight_time)
            theta = arc_length / R
            
            # 弯道起点：直道末端 (1000, 0)
            # 弯道中心：在直道右侧500米处 (1000, 500)
            center_x = straight_length
            center_y = R
            
            # 计算弯道上的位置
            # 向右转弯：从(1000,0)开始，向上弯曲
            x = center_x + R * np.sin(theta)
            y = center_y - R * np.cos(theta)
            psi_ref = theta
            curvature = 1.0 / R
            
        return x, y, psi_ref, curvature

def generate_lane_lines(ref_x, ref_y, lane_width=LANE_WIDTH):
    """
    根据参考路径生成左右车道线
    """
    # 计算路径的切向量
    dx = np.gradient(ref_x)
    dy = np.gradient(ref_y)
    
    # 归一化切向量
    norm = np.sqrt(dx**2 + dy**2)
    dx = dx / norm
    dy = dy / norm
    
    # 计算法向量（垂直于切向量）
    nx = -dy
    ny = dx
    
    # 生成左右车道线
    left_x = ref_x + nx * lane_width / 2
    left_y = ref_y + ny * lane_width / 2
    right_x = ref_x - nx * lane_width / 2
    right_y = ref_y - ny * lane_width / 2
    
    return left_x, left_y, right_x, right_y

def test_trajectory():
    """测试轨迹生成"""
    v = 10.0  # 10 m/s
    sim_time = 150.0  # 150秒
    dt = 0.1
    
    # 生成轨迹
    time_points = np.arange(0, sim_time, dt)
    ref_x = []
    ref_y = []
    psi_ref = []
    curvature = []
    
    for t in time_points:
        x, y, psi, curv = reference_path_highway(t, v, 'highway_curve')
        ref_x.append(x)
        ref_y.append(y)
        psi_ref.append(psi)
        curvature.append(curv)
    
    ref_x = np.array(ref_x)
    ref_y = np.array(ref_y)
    psi_ref = np.array(psi_ref)
    curvature = np.array(curvature)
    
    # 生成车道线
    left_x, left_y, right_x, right_y = generate_lane_lines(ref_x, ref_y)
    
    # 绘制结果
    plt.figure(figsize=(15, 10))
    
    # 轨迹图
    plt.subplot(2, 2, 1)
    plt.plot(ref_x, ref_y, 'r-', linewidth=3, label='Reference Path (Center Line)')
    plt.plot(left_x, left_y, 'k--', linewidth=2, label='Left Lane')
    plt.plot(right_x, right_y, 'k--', linewidth=2, label='Right Lane')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Highway Trajectory: 1000m Straight + 500m Radius Curve')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    # 偏航角
    plt.subplot(2, 2, 2)
    plt.plot(time_points, psi_ref, 'b-', linewidth=2)
    plt.xlabel('Time [s]')
    plt.ylabel('Yaw Angle [rad]')
    plt.title('Reference Yaw Angle')
    plt.grid(True)
    
    # 曲率
    plt.subplot(2, 2, 3)
    plt.plot(time_points, curvature, 'g-', linewidth=2)
    plt.xlabel('Time [s]')
    plt.ylabel('Curvature [1/m]')
    plt.title('Path Curvature')
    plt.grid(True)
    
    # 轨迹参数
    plt.subplot(2, 2, 4)
    plt.text(0.1, 0.8, f'Lane Width: {LANE_WIDTH} m', fontsize=12, transform=plt.gca().transAxes)
    plt.text(0.1, 0.6, f'Straight Length: 1000 m', fontsize=12, transform=plt.gca().transAxes)
    plt.text(0.1, 0.4, f'Curve Radius: 500 m', fontsize=12, transform=plt.gca().transAxes)
    plt.text(0.1, 0.2, f'Vehicle Speed: {v} m/s', fontsize=12, transform=plt.gca().transAxes)
    plt.title('Trajectory Parameters')
    plt.axis('off')
    
    plt.tight_layout()
    plt.savefig('trajectory_test_result.png', dpi=300, bbox_inches='tight')
    
    # 打印关键信息
    print("=== 轨迹测试结果 ===")
    print(f"车道宽度: {LANE_WIDTH} m")
    print(f"直道长度: 1000 m")
    print(f"弯道半径: 500 m")
    print(f"车辆速度: {v} m/s")
    print(f"仿真时间: {sim_time} s")
    
    # 检查轨迹连续性
    straight_length = 1000.0
    straight_end_idx = int(straight_length / v / dt)
    straight_end_x = ref_x[straight_end_idx]
    straight_end_y = ref_y[straight_end_idx]
    curve_start_x = ref_x[straight_end_idx + 1]
    curve_start_y = ref_y[straight_end_idx + 1]
    
    print(f"\n=== 轨迹连续性检查 ===")
    print(f"直道末端位置: ({straight_end_x:.1f}, {straight_end_y:.1f})")
    print(f"弯道起始位置: ({curve_start_x:.1f}, {curve_start_y:.1f})")
    print(f"位置差异: dx={abs(straight_end_x - curve_start_x):.3f}, dy={abs(straight_end_y - curve_start_y):.3f}")
    
    if abs(straight_end_x - curve_start_x) < 0.1 and abs(straight_end_y - curve_start_y) < 0.1:
        print("✅ 轨迹连续性良好！")
    else:
        print("❌ 轨迹连续性有问题！")
        
        # 分析问题
        print(f"\n=== 问题分析 ===")
        print(f"直道时间: {straight_length / v:.1f} s")
        print(f"直道结束索引: {straight_end_idx}")
        print(f"时间步长: {dt} s")
        
        # 手动计算弯道起始位置
        R = 500.0
        theta_start = 0.0
        center_x = straight_length
        center_y = R
        manual_curve_x = center_x + R * np.sin(theta_start)
        manual_curve_y = center_y - R * np.cos(theta_start)
        
        print(f"手动计算弯道起始: ({manual_curve_x:.1f}, {manual_curve_y:.1f})")
        print(f"应该的弯道起始: ({straight_length:.1f}, {0.0:.1f})")

if __name__ == "__main__":
    test_trajectory() 