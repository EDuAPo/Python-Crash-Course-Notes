import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from scipy import linalg
import matplotlib.animation as animation

# =========================
# 可调整参数列表
# =========================
CONTROL_PARAMS = {
    # 几何控制器参数
    'k_e': 0.05,           # 横向误差增益
    'k_psi': 0.3,          # 偏航角误差增益
    'max_steer': 0.1,      # 最大转向角限制
    
    # 前馈控制参数
    'k_ff': 2.0,           # 前馈增益（增大以改善入弯）
    'k_curvature': 1.5,    # 曲率前馈增益
    
    # 车辆参数
    'speed': 1.0,          # 车速 [m/s]
    'sim_time': 1200.0,    # 仿真时间 [s]
    
    # 轨迹参数
    'straight_length': 1000.0,  # 直道长度 [m]
    'curve_radius': 500.0,      # 弯道半径 [m]
    'lane_width': 3.8,          # 车道宽度 [m]
    
    # 可视化参数
    'margin_x': 50,        # 横轴边距
    'margin_y': 20,        # 纵轴边距
    'grid_scale': 0.5,     # 网格比例尺 [m]
}

# =========================
# 完整的车辆参数（基于真实车辆）
# =========================
CAR_PARAMS = {
    'm': 1500.0,      # 质量 [kg]
    'Iz': 2500.0,     # 绕质心的惯性矩 [kg*m^2]
    'lf': 1.2,        # 前轴到质心距离 [m]
    'lr': 1.6,        # 后轴到质心距离 [m]
    'Cf': 80000.0,    # 前轮侧偏刚度 [N/rad]
    'Cr': 80000.0,    # 后轮侧偏刚度 [N/rad]
    'wheelbase': 2.8  # 轴距 [m]
}

# 车道参数
LANE_WIDTH = CONTROL_PARAMS['lane_width']

# =========================
# 完整的车辆横向动力学模型（自行车模型）
# =========================
def get_state_space_model(v, params):
    """
    构建完整的线性化自行车模型的状态空间表达式
    状态变量: [横向位置误差 e, 横向速度误差 e_dot, 偏航角误差 psi, 偏航角速度误差 psi_dot]
    """
    m = params['m']
    Iz = params['Iz']
    lf = params['lf']
    lr = params['lr']
    Cf = params['Cf']
    Cr = params['Cr']

    if v < 1.0:
        v = 1.0

    # 计算系数
    a11 = -(2*Cf + 2*Cr) / (m * v)
    a12 = (2*Cf + 2*Cr) / m
    a13 = -(2*Cf*lf - 2*Cr*lr) / (m * v)
    a31 = -(2*Cf*lf - 2*Cr*lr) / (Iz * v)
    a32 = (2*Cf*lf - 2*Cr*lr) / Iz
    a33 = -(2*Cf*lf**2 + 2*Cr*lr**2) / (Iz * v)
    
    b1 = 2*Cf / m
    b3 = 2*Cf*lf / Iz

    A = np.array([
        [0, 1, 0, 0],
        [0, a11, a12, a13],
        [0, 0, 0, 1],
        [0, a31, a32, a33]
    ])
    
    B = np.array([[0], [b1], [0], [b3]])

    return A, B

# =========================
# 改进的LQR控制器
# =========================
def solve_lqr_improved(A, B, Q, R):
    """
    改进的LQR求解，包含错误处理
    """
    try:
        # 检查系统可控性
        n = A.shape[0]
        controllability_matrix = np.hstack([B] + [np.linalg.matrix_power(A, i) @ B for i in range(1, n)])
        if np.linalg.matrix_rank(controllability_matrix) < n:
            print("警告：系统不可控，使用备用控制器")
            return np.array([[5.0, 1.0, 3.0, 0.5]])
        
        # 求解连续黎卡提方程
        P = linalg.solve_continuous_are(A, B, Q, R)
        K = np.dot(np.linalg.inv(R), np.dot(B.T, P))
        return K
    except Exception as e:
        print(f"LQR求解失败: {e}，使用备用控制器")
        return np.array([[5.0, 1.0, 3.0, 0.5]])

# =========================
# 改进的Stanley控制器
# =========================
class ImprovedStanleyController:
    """
    改进的Stanley控制器：包含速度自适应和饱和处理
    """
    def __init__(self, k_e=1.0, k_psi=1.0, k_v=0.1, max_steer=0.5):
        self.k_e = k_e
        self.k_psi = k_psi
        self.k_v = k_v
        self.max_steer = max_steer
    
    def control(self, e, psi_err, v, psi_ref):
        """
        改进的Stanley控制律
        """
        if abs(v) < 0.1:
            v = 0.1
        
        # 速度自适应增益
        k_e_adaptive = self.k_e * (1.0 + 0.1 * v)
        
        # Stanley控制律
        delta = psi_err + np.arctan(k_e_adaptive * e / (self.k_v + v))
        
        # 饱和处理
        delta = np.clip(delta, -self.max_steer, self.max_steer)
        
        return delta

# =========================
# 完整的车辆动力学模型
# =========================
def vehicle_dynamics_complete(x, y, psi, v, delta, dt, params):
    """
    完整的车辆动力学模型（运动学+动力学）
    """
    L = params['wheelbase']
    m = params['m']
    Iz = params['Iz']
    lf = params['lf']
    lr = params['lr']
    Cf = params['Cf']
    Cr = params['Cr']
    
    if abs(v) < 0.1:
        v = 0.1
    
    # 计算侧偏角
    beta = np.arctan((lr * np.tan(delta)) / L)
    
    # 计算侧向力
    alpha_f = delta - beta - lf * psi / v
    alpha_r = -beta + lr * psi / v
    
    Fyf = Cf * alpha_f
    Fyr = Cr * alpha_r
    
    # 动力学方程
    y_ddot = (Fyf + Fyr) / m - v * psi
    psi_ddot = (lf * Fyf - lr * Fyr) / Iz
    
    # 积分更新
    y_dot = y_ddot * dt
    psi_dot = psi_ddot * dt
    
    # 位置更新
    x_new = x + v * np.cos(psi) * dt
    y_new = y + v * np.sin(psi) * dt
    psi_new = psi + psi_dot
    
    return x_new, y_new, psi_new

# =========================
# 车道线生成函数
# =========================
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

# =========================
# 修正的1000米直道+500米弯道轨迹生成
# =========================
def reference_path_highway(t, v, path_type='highway_curve', control_params=None):
    """
    高速公路轨迹：可调整的直道长度 + 弯道半径
    修正版本：确保轨迹连续性
    """
    if control_params is None:
        control_params = CONTROL_PARAMS
    
    straight_length = control_params['straight_length']
    curve_radius = control_params['curve_radius']
    
    if path_type == 'highway_curve':
        # 直道段
        straight_time = straight_length / v
        
        if t <= straight_time:
            # 直道段：沿x轴正方向
            x = v * t
            y = 0.0
            psi_ref = 0.0
            curvature = 0.0
        else:
            # 弯道段：从直道末端开始，向右转弯
            arc_length = v * (t - straight_time)
            theta = arc_length / curve_radius
            
            # 弯道起点：直道末端 (straight_length, 0)
            # 弯道中心：在直道右侧curve_radius米处 (straight_length, curve_radius)
            center_x = straight_length
            center_y = curve_radius
            
            # 计算弯道上的位置
            # 向右转弯：从(straight_length,0)开始，向上弯曲
            x = center_x + curve_radius * np.sin(theta)
            y = center_y - curve_radius * np.cos(theta)
            psi_ref = theta
            curvature = 1.0 / curve_radius
            
        return x, y, psi_ref, curvature
    
    elif path_type == 'straight_only':
        # 纯直道轨迹
        x = v * t
        y = 0.0
        psi_ref = 0.0
        curvature = 0.0
        return x, y, psi_ref, curvature
    
    elif path_type == 'straight_curve':
        # 前20米直道，然后接一个缓和的圆弧
        short_straight = 20.0
        short_straight_time = short_straight / v
        
        if t <= short_straight_time:
            # 直道段
            x = v * t
            y = 0.0
            psi_ref = 0.0
            curvature = 0.0
        else:
            # 圆弧段 (半径50米)
            R = 50.0
            arc_length = v * (t - short_straight_time)
            theta = arc_length / R
            x = short_straight + R * np.sin(theta)
            y = R * (1 - np.cos(theta))
            psi_ref = theta
            curvature = 1.0 / R
            
        return x, y, psi_ref, curvature
    
    elif path_type == 'sine_simple':
        # 简单的正弦曲线
        x = v * t
        y = 2.0 * np.sin(0.1 * x)  # 振幅2m，波长约63m
        psi_ref = np.arctan(0.2 * np.cos(0.1 * x))
        curvature = -0.02 * np.sin(0.1 * x) / (1 + (0.2 * np.cos(0.1 * x))**2)**1.5
        return x, y, psi_ref, curvature
    
    elif path_type == 'circle_large':
        # 大半径圆形路径
        R = 100.0  # 增大半径
        theta = v * t / R
        x = R * np.sin(theta)
        y = R * (1 - np.cos(theta))
        psi_ref = theta
        curvature = 1.0 / R
        return x, y, psi_ref, curvature
    
    else:
        raise ValueError("未知路径类型: {}".format(path_type))

# =========================
# 改进的前馈控制
# =========================
def feedforward_control_improved(v, curvature, params, control_params):
    """
    改进的前馈控制：考虑车辆动力学特性，使用可调整参数
    """
    L = params['wheelbase']
    m = params['m']
    Cf = params['Cf']
    Cr = params['Cr']
    lf = params['lf']
    lr = params['lr']
    
    # 使用可调整的前馈参数
    k_ff = control_params['k_ff']
    k_curvature = control_params['k_curvature']
    
    if np.abs(curvature) > 1e-6:
        # 考虑车辆动力学的前馈控制，增大前馈增益
        Kv = k_curvature * (lr * m / Cf / L - lf * m / Cr / L)
        delta_ff = k_ff * (L * curvature + Kv * v**2 * curvature)
    else:
        delta_ff = 0.0
    return delta_ff

# =========================
# 动态图显示类
# =========================
class DynamicLCCVisualizer:
    """
    动态LCC可视化类
    """
    def __init__(self, lane_width=LANE_WIDTH):
        self.lane_width = lane_width
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.vehicle_trajectory = []
        self.ref_trajectory = []
        self.left_lane = []
        self.right_lane = []
        
    def init_animation(self):
        """初始化动画"""
        self.ax.clear()
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title('LCC Dynamic Simulation - Lane Centering Control')
        self.ax.grid(True, alpha=0.3)
        return []
    
    def update_animation(self, frame, results, ref_x, ref_y):
        """更新动画帧"""
        self.ax.clear()
        
        # 设置坐标轴范围 - 确保显示完整轨迹
        max_x = max(np.max(ref_x), np.max(results['x']))
        min_x = min(np.min(ref_x), np.min(results['x']))
        max_y = max(np.max(ref_y), np.max(results['y']))
        min_y = min(np.min(ref_y), np.min(results['y']))
        
        # 添加边距
        margin_x = CONTROL_PARAMS['margin_x']
        margin_y = CONTROL_PARAMS['margin_y']
        self.ax.set_xlim(min_x - margin_x, max_x + margin_x)
        self.ax.set_ylim(min_y - margin_y, max_y + margin_y)
        
        # 自适应网格：横轴自适应，纵轴使用0.5米网格
        self.ax.set_yticks(np.arange(min_y - margin_y, max_y + margin_y + 0.5, 0.5))
        self.ax.grid(True, alpha=0.3, linewidth=0.5)
        
        # 绘制参考路径
        self.ax.plot(ref_x, ref_y, 'r--', linewidth=2, label='Reference Path', alpha=0.7)
        
        # 生成车道线
        left_x, left_y, right_x, right_y = generate_lane_lines(ref_x, ref_y, self.lane_width)
        
        # 绘制车道线
        self.ax.plot(left_x, left_y, 'k-', linewidth=3, label='Left Lane', alpha=0.8)
        self.ax.plot(right_x, right_y, 'k-', linewidth=3, label='Right Lane', alpha=0.8)
        
        # 绘制车辆轨迹（到当前帧）
        if frame > 0:
            self.ax.plot(results['x'][:frame], results['y'][:frame], 'b-', linewidth=2, label='Vehicle Trajectory', alpha=0.8)
        
        # 绘制当前车辆位置
        if frame < len(results['x']):
            vehicle_x = results['x'][frame]
            vehicle_y = results['y'][frame]
            vehicle_psi = results['psi'][frame]
            
            # 绘制车辆（简化为矩形）
            car_length = 4.0
            car_width = 2.0
            
            # 车辆四个角点
            corners = np.array([
                [-car_length/2, -car_width/2],
                [car_length/2, -car_width/2],
                [car_length/2, car_width/2],
                [-car_length/2, car_width/2]
            ])
            
            # 旋转车辆
            cos_psi = np.cos(vehicle_psi)
            sin_psi = np.sin(vehicle_psi)
            rotation_matrix = np.array([[cos_psi, -sin_psi], [sin_psi, cos_psi]])
            
            rotated_corners = corners @ rotation_matrix.T
            rotated_corners[:, 0] += vehicle_x
            rotated_corners[:, 1] += vehicle_y
            
            # 绘制车辆
            vehicle_polygon = plt.Polygon(rotated_corners, facecolor='blue', alpha=0.7, edgecolor='black')
            self.ax.add_patch(vehicle_polygon)
            
            # 绘制车辆中心点
            self.ax.plot(vehicle_x, vehicle_y, 'ko', markersize=8, label='Vehicle Position')
            
            # 显示当前信息
            lateral_error = results['e'][frame]
            steering_angle = results['delta'][frame]
            
            info_text = f'Time: {results["time"][frame]:.1f}s\nLateral Error: {lateral_error:.3f}m\nSteering: {steering_angle:.3f}rad'
            self.ax.text(0.02, 0.98, info_text, transform=self.ax.transAxes, 
                        verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title(f'LCC Dynamic Simulation - Lane Centering Control (Frame: {frame}/{len(results["x"])})')
        self.ax.legend(loc='upper left')
        
        return []
    
    def create_animation(self, results, ref_x, ref_y, save_path='lcc_animation.gif'):
        """创建动画"""
        frames = len(results['x'])
        # 增加帧数以显示更多细节，特别是弯道部分
        step = max(1, frames // 200)  # 最多200帧，显示更多细节
        frames_to_show = range(0, frames, step)
        
        anim = animation.FuncAnimation(
            self.fig, 
            self.update_animation, 
            frames=frames_to_show,
            fargs=(results, ref_x, ref_y),
            init_func=self.init_animation,
            interval=80,  # 80ms per frame，稍微快一点
            blit=False,
            repeat=True
        )
        
        # 保存动画
        print(f"Saving animation to {save_path}...")
        try:
            anim.save(save_path, writer='pillow', fps=12)  # 提高帧率
            print("Animation saved successfully!")
        except Exception as e:
            print(f"Animation save failed: {e}")
            # 保存静态图作为备选
            self.update_animation(frames-1, results, ref_x, ref_y)
            plt.savefig('lcc_static_result.png', dpi=300, bbox_inches='tight')
            print("Static result image saved as lcc_static_result.png")
        
        return anim

# =========================
# 改进的几何控制器（包含前馈控制）
# =========================
class ImprovedGeometricController:
    """
    改进的几何控制器：包含前馈控制和可调整参数
    """
    def __init__(self, control_params):
        self.k_e = control_params['k_e']
        self.k_psi = control_params['k_psi']
        self.max_steer = control_params['max_steer']
        self.k_ff = control_params['k_ff']
        self.k_curvature = control_params['k_curvature']
    
    def control(self, e, psi_err, v, curvature, params):
        """
        改进的几何控制律：包含前馈控制
        """
        # 反馈控制
        delta_fb = self.k_psi * psi_err + self.k_e * e
        
        # 前馈控制
        delta_ff = feedforward_control_improved(v, curvature, params, {
            'k_ff': self.k_ff,
            'k_curvature': self.k_curvature
        })
        
        # 总控制输入
        delta = delta_fb + delta_ff
        
        # 饱和处理
        delta = np.clip(delta, -self.max_steer, self.max_steer)
        
        return delta

# =========================
# 最简单的几何控制器
# =========================
class PureGeometricController:
    """
    最简单的几何控制器：纯比例控制
    """
    def __init__(self, k_e=0.1, k_psi=0.5, max_steer=0.2):
        self.k_e = k_e
        self.k_psi = k_psi
        self.max_steer = max_steer
    
    def control(self, e, psi_err, v):
        """
        最简单的几何控制律
        """
        # 纯比例控制
        delta = self.k_psi * psi_err + self.k_e * e
        
        # 饱和处理
        delta = np.clip(delta, -self.max_steer, self.max_steer)
        
        return delta

# =========================
# 最简单的车辆模型
# =========================
def vehicle_dynamics_simple(x, y, psi, v, delta, dt, params):
    """
    最简单的车辆模型：纯积分器
    """
    L = params['wheelbase']
    
    if abs(v) < 0.1:
        v = 0.1
    
    # 最简单的运动学方程
    x_new = x + v * np.cos(psi) * dt
    y_new = y + v * np.sin(psi) * dt
    psi_new = psi + (v * delta / L) * dt  # 线性化转向关系
    
    return x_new, y_new, psi_new

# =========================
# 改进的仿真主循环（使用可调整参数）
# =========================
def simulate_lane_centering_improved(params, control_params=None, path_type='highway_curve'):
    """
    使用改进模型的车道居中控制仿真
    控制目标：车道中心线
    """
    if control_params is None:
        control_params = CONTROL_PARAMS
    
    # 初始化车辆状态 - 放在车道中心线上
    x_vehicle = 0.0
    y_vehicle = 0.0  # 放在车道中心线上，无初始偏移
    psi_vehicle = 0.0
    
    # 初始化改进的控制器
    controller = ImprovedGeometricController(control_params)
    
    # 获取仿真参数
    sim_time = control_params['sim_time']
    dt = 0.01
    v = control_params['speed']
    
    # 记录数据
    time_hist = []
    x_hist = []
    y_hist = []
    e_hist = []
    psi_hist = []
    delta_hist = []
    ref_x_hist = []
    ref_y_hist = []
    
    steps = int(sim_time / dt)
    for step in range(steps):
        t = step * dt
        
        # 参考路径（车道中心线）
        ref_x, ref_y, psi_ref, curvature = reference_path_highway(t, v, path_type, control_params)
        ref_x_hist.append(ref_x)
        ref_y_hist.append(ref_y)
        
        # 计算横向误差（相对于车道中心线）
        dx = x_vehicle - ref_x
        dy = y_vehicle - ref_y
        e = np.sin(psi_ref) * dx - np.cos(psi_ref) * dy
        
        # 计算偏航角误差
        psi_err = psi_vehicle - psi_ref
        psi_err = np.arctan2(np.sin(psi_err), np.cos(psi_err))
        
        # 改进的几何控制（包含前馈）
        delta = controller.control(e, psi_err, v, curvature, params)
        
        # 车辆动力学更新
        x_vehicle, y_vehicle, psi_vehicle = vehicle_dynamics_simple(
            x_vehicle, y_vehicle, psi_vehicle, v, delta, dt, params
        )
        
        # 记录数据
        time_hist.append(t)
        x_hist.append(x_vehicle)
        y_hist.append(y_vehicle)
        e_hist.append(e)
        psi_hist.append(psi_vehicle)
        delta_hist.append(delta)
        
        if step % 1000 == 0:
            print(f"时间: {t:.1f}s, 位置: ({x_vehicle:.1f}, {y_vehicle:.1f}), 横向误差: {e:.3f}m, 控制输入: {delta:.3f}rad")
    
    return {
        'time': np.array(time_hist),
        'x': np.array(x_hist),
        'y': np.array(y_hist),
        'e': np.array(e_hist),
        'psi': np.array(psi_hist),
        'delta': np.array(delta_hist),
        'ref_x': np.array(ref_x_hist),
        'ref_y': np.array(ref_y_hist)
    }

# =========================
# 可视化
# =========================
def plot_results(results):
    plt.figure(figsize=(15, 10))
    
    # 轨迹图
    plt.subplot(2, 3, 1)
    plt.plot(results['ref_x'], results['ref_y'], 'r--', label='Reference Path', linewidth=2)
    plt.plot(results['x'], results['y'], 'b-', label='Vehicle Trajectory', linewidth=1)
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Vehicle Path Tracking')
    plt.legend()
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    
    # 自适应网格：横轴自适应，纵轴使用0.5米网格
    max_y = max(np.max(results['ref_y']), np.max(results['y']))
    min_y = min(np.min(results['ref_y']), np.min(results['y']))
    margin_y = CONTROL_PARAMS['margin_y']
    
    plt.yticks(np.arange(min_y - margin_y, max_y + margin_y + 0.5, 0.5))
    
    # 横向误差
    plt.subplot(2, 3, 2)
    plt.plot(results['time'], results['e'], 'g-', linewidth=2, label='Lateral Error')
    plt.xlabel('Time [s]')
    plt.ylabel('Lateral Error [m]')
    plt.title('Lateral Error')
    plt.axhline(y=0.5, color='r', linestyle='--', label='0.5m limit')
    plt.axhline(y=-0.5, color='r', linestyle='--')
    plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # 设置0.5米网格
    plt.yticks(np.arange(-1.0, 1.1, 0.5))
    
    # 转向角
    plt.subplot(2, 3, 3)
    plt.plot(results['time'], results['delta'], 'm-', linewidth=2)
    plt.xlabel('Time [s]')
    plt.ylabel('Steering Angle [rad]')
    plt.title('Steering Input')
    plt.grid(True, alpha=0.3)
    
    # 偏航角
    plt.subplot(2, 3, 4)
    plt.plot(results['time'], results['psi'], 'c-', linewidth=2)
    plt.xlabel('Time [s]')
    plt.ylabel('Yaw Angle [rad]')
    plt.title('Vehicle Yaw Angle')
    plt.grid(True, alpha=0.3)
    
    # 横向误差分布
    plt.subplot(2, 3, 5)
    plt.hist(results['e'], bins=50, alpha=0.7, color='green')
    plt.xlabel('Lateral Error [m]')
    plt.ylabel('Frequency')
    plt.title('Lateral Error Distribution')
    plt.axvline(x=0.5, color='r', linestyle='--', label='0.5m limit')
    plt.axvline(x=-0.5, color='r', linestyle='--')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # 设置0.5米网格
    plt.xticks(np.arange(-1.0, 1.1, 0.5))
    
    # 误差统计
    plt.subplot(2, 3, 6)
    max_error = np.max(np.abs(results['e']))
    mean_error = np.mean(np.abs(results['e']))
    std_error = np.std(results['e'])
    
    plt.text(0.1, 0.8, f'Max Error: {max_error:.3f} m', fontsize=12, transform=plt.gca().transAxes)
    plt.text(0.1, 0.6, f'Mean Error: {mean_error:.3f} m', fontsize=12, transform=plt.gca().transAxes)
    plt.text(0.1, 0.4, f'Std Error: {std_error:.3f} m', fontsize=12, transform=plt.gca().transAxes)
    plt.text(0.1, 0.2, f'Success: {"✅" if max_error < 0.5 else "❌"}', fontsize=12, transform=plt.gca().transAxes)
    plt.title('Error Statistics')
    plt.axis('off')
    
    plt.tight_layout()
    plt.savefig('lcc_simulation_results.png', dpi=300, bbox_inches='tight')
    plt.close()

# =========================
# 主函数
# =========================
def main():
    try:
        print("开始改进的LCC仿真...")
        print("轨迹：{}米直道 + 半径{}米弯道".format(
            CONTROL_PARAMS['straight_length'], CONTROL_PARAMS['curve_radius']))
        print("车道宽度：{}米".format(CONTROL_PARAMS['lane_width']))
        print("速度：{} m/s ({:.1f} km/h)".format(CONTROL_PARAMS['speed'], CONTROL_PARAMS['speed']*3.6))
        print("模型：改进几何模型 + 前馈控制")
        print("控制目标：车道中心线")
        print("初始位置：车道中心线上")
        
        print("\n=== 当前控制参数 ===")
        print("几何控制器参数:")
        print(f"  k_e: {CONTROL_PARAMS['k_e']} (横向误差增益)")
        print(f"  k_psi: {CONTROL_PARAMS['k_psi']} (偏航角误差增益)")
        print(f"  max_steer: {CONTROL_PARAMS['max_steer']} (最大转向角限制)")
        print("前馈控制参数:")
        print(f"  k_ff: {CONTROL_PARAMS['k_ff']} (前馈增益)")
        print(f"  k_curvature: {CONTROL_PARAMS['k_curvature']} (曲率前馈增益)")
        
        # 测试高速公路轨迹
        print("\n=== 测试高速公路轨迹 ===")
        print(f"仿真时间：{CONTROL_PARAMS['sim_time']:.0f}秒")
        results = simulate_lane_centering_improved(CAR_PARAMS, CONTROL_PARAMS, 'highway_curve')
        
        if results is not None:
            # 保存静态结果图
            plot_results(results)
            plt.savefig('lcc_dynamic_results.png', dpi=300, bbox_inches='tight')
            plt.close()
            
            max_lateral_error = np.max(np.abs(results['e']))
            mean_lateral_error = np.mean(np.abs(results['e']))
            std_lateral_error = np.std(results['e'])
            
            print(f"\n=== 仿真结果 ===")
            print(f"最大横向误差: {max_lateral_error:.3f} m")
            print(f"平均横向误差: {mean_lateral_error:.3f} m")
            print(f"横向误差标准差: {std_lateral_error:.3f} m")
            
            if max_lateral_error < 0.5:
                print("✅ 横向误差控制成功！最大误差小于0.5米")
            else:
                print("❌ 横向误差超出要求！")
                print("建议调整参数：")
                print("  1. 增大 k_ff (前馈增益)")
                print("  2. 增大 k_curvature (曲率前馈增益)")
                print("  3. 调整 k_e 和 k_psi (反馈增益)")
            
            # 创建动态图
            print("\n=== 创建动态图 ===")
            visualizer = DynamicLCCVisualizer()
            anim = visualizer.create_animation(results, results['ref_x'], results['ref_y'], 'lcc_animation.gif')
            
            print("动态图创建完成！")
            
        else:
            print("仿真未能成功运行。")
                
    except Exception as e:
        print("仿真过程中发生错误:", e)
        import traceback
        traceback.print_exc()

# =========================
# 参数调整示例
# =========================
def adjust_parameters_example():
    """
    参数调整示例：展示如何手动调整控制参数
    """
    print("=== 参数调整示例 ===")
    print("当前参数设置：")
    for key, value in CONTROL_PARAMS.items():
        print(f"  {key}: {value}")
    
    print("\n=== 参数调整建议 ===")
    print("1. 增大前馈增益以改善入弯性能：")
    print("   CONTROL_PARAMS['k_ff'] = 3.0  # 从2.0增加到3.0")
    print("   CONTROL_PARAMS['k_curvature'] = 2.0  # 从1.5增加到2.0")
    
    print("\n2. 调整反馈增益以改善跟踪性能：")
    print("   CONTROL_PARAMS['k_e'] = 0.1  # 从0.05增加到0.1")
    print("   CONTROL_PARAMS['k_psi'] = 0.5  # 从0.3增加到0.5")
    
    print("\n3. 增大最大转向角限制：")
    print("   CONTROL_PARAMS['max_steer'] = 0.2  # 从0.1增加到0.2")
    
    print("\n4. 调整车速：")
    print("   CONTROL_PARAMS['speed'] = 0.5  # 降低车速以改善控制性能")
    
    print("\n=== 使用示例 ===")
    print("# 创建自定义参数")
    print("custom_params = CONTROL_PARAMS.copy()")
    print("custom_params['k_ff'] = 3.0")
    print("custom_params['k_curvature'] = 2.0")
    print("custom_params['k_e'] = 0.1")
    print("custom_params['k_psi'] = 0.5")
    print("custom_params['max_steer'] = 0.2")
    print("custom_params['speed'] = 0.5")
    print("")
    print("# 运行仿真")
    print("results = simulate_lane_centering_improved(CAR_PARAMS, custom_params, 'highway_curve')")

def run_with_custom_params():
    """
    使用自定义参数运行仿真的示例
    """
    print("=== 使用自定义参数运行仿真 ===")
    
    # 创建自定义参数
    custom_params = CONTROL_PARAMS.copy()
    custom_params['k_ff'] = 3.0  # 增大前馈增益
    custom_params['k_curvature'] = 2.0  # 增大曲率前馈增益
    custom_params['k_e'] = 0.1  # 增大横向误差增益
    custom_params['k_psi'] = 0.5  # 增大偏航角误差增益
    custom_params['max_steer'] = 0.2  # 增大最大转向角限制
    custom_params['speed'] = 0.5  # 降低车速
    
    print("自定义参数：")
    for key, value in custom_params.items():
        print(f"  {key}: {value}")
    
    # 运行仿真
    results = simulate_lane_centering_improved(CAR_PARAMS, custom_params, 'highway_curve')
    
    if results is not None:
        max_lateral_error = np.max(np.abs(results['e']))
        mean_lateral_error = np.mean(np.abs(results['e']))
        std_lateral_error = np.std(results['e'])
        
        print(f"\n=== 自定义参数仿真结果 ===")
        print(f"最大横向误差: {max_lateral_error:.3f} m")
        print(f"平均横向误差: {mean_lateral_error:.3f} m")
        print(f"横向误差标准差: {std_lateral_error:.3f} m")
        
        if max_lateral_error < 0.5:
            print("✅ 横向误差控制成功！最大误差小于0.5米")
        else:
            print("❌ 横向误差超出要求！")
        
        # 创建动态图
        print("\n=== 创建自定义参数动态图 ===")
        visualizer = DynamicLCCVisualizer()
        anim = visualizer.create_animation(results, results['ref_x'], results['ref_y'], 'lcc_custom_animation.gif')
        print("自定义参数动态图创建完成！")
    
    return results

if __name__ == "__main__":
    # 显示参数调整示例
    adjust_parameters_example()
    
    # 询问是否运行自定义参数仿真
    print("\n是否运行自定义参数仿真？(y/n): ", end="")
    # 这里可以添加用户输入，现在直接运行
    print("y")
    
    # 运行自定义参数仿真
    custom_results = run_with_custom_params()
    
    # 运行默认参数仿真
    print("\n" + "="*50)
    main()
