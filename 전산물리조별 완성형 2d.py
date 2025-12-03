import pygame
import numpy as np
import sys
import matplotlib
matplotlib.use('Agg')  # GUI 백엔드 비활성화 (충돌 방지)
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
import io

# Pygame 초기화
pygame.init()

# 화면 설정
WIDTH, HEIGHT = 1200, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Port Crane Simulation - Final Physics Corrected")
clock = pygame.time.Clock()

# 색상 정의
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 100, 255)
GRAY = (150, 150, 150)
GREEN = (0, 200, 0)
ORANGE = (255, 140, 0)
DARK_GRAY = (80, 80, 80)
PURPLE = (150, 0, 150)
CYAN = (0, 200, 200)

# 물리 상수
g = 9.81   # 중력 가속도 (m/s^2)
L_total = 20.0  # 케이블 총 길이 (m)
m = 15000.0  # 컨테이너 질량 (kg)
SCALE = 80  # 픽셀/미터 변환 비율

# 40ft 컨테이너 실제 치수 (m)
CONTAINER_LENGTH = 12.192
CONTAINER_WIDTH = 2.438
CONTAINER_HEIGHT = 2.591
CONTAINER_PIXEL_WIDTH = 60
CONTAINER_PIXEL_HEIGHT = 40

# 공기저항 파라미터
rho_air = 1.225
C_d = 1.05
A_frontal = CONTAINER_WIDTH * CONTAINER_HEIGHT

# 초기 가속도 실험 파라미터
INITIAL_ACCEL = 1.5
ACCEL_DURATION = 1.0

# 제어 파라미터
Kp = 30.0
Kd = 20.0

# --- [초기 설정] 상호 연동 ---
target_cable_length = 3.0  # 목표 케이블 길이 (m)
target_height = L_total - target_cable_length  # 목표 높이 (17.0m)

lifting_speed = 2.0   # 들어올리는 속도 (m/s)

# 상태 변수
L = L_total  # 바닥(20m)에서 시작
is_lifting = True
lifting_completed = False

# 최적화 탐색 관련
DISP_LIMIT = 0.05
TOTAL_TIME = 15.0
A_MIN = 0.0
A_MAX = 2.0
DA = 0.01

# 크레인 상태
crane_x = WIDTH // 2
crane_v = 0.0
external_accel = 0.0

# 진자 상태
theta = 0.0
omega = 0.0

dt = 0.02

# 폰트
font = pygame.font.Font(None, 24)
large_font = pygame.font.Font(None, 36)

# 데이터 기록
time_elapsed = 0.0
force_history = []
angle_history = []
energy_history = []

# 실험 상태
trial_running = False
trial_phase = "idle"
trial_time = 0.0

# 결과값
max_angle_deg = 0.0
max_horizontal_disp = 0.0
max_tension = 0.0
max_control_torque = 0.0
has_results = False

# 에너지
lifting_energy = 0.0
control_energy_net = 0.0
control_energy_pos = 0.0
control_energy_neg = 0.0
total_energy_required = 0.0
cumulative_kinetic_energy = 0.0

# 입력창
input_length_text = ""
length_input_active = False
length_rect = None

input_height_text = ""
height_input_active = False
height_rect = None

# 기타 UI 변수
show_opt_info = False
opt_accel_found = 0.0
show_matplotlib_graph = False
graph_surface = None
show_length_analysis_graph = False
length_analysis_surface = None
length_analysis_data = []
lifting_msg_shown = False


def create_matplotlib_graph():
    """에너지 상세 분석 그래프 생성 (시간축)"""
    global graph_surface
    if len(energy_history) < 2: return
    
    times = [h['time'] for h in energy_history]
    lifting_energies = [h['lifting_energy'] for h in energy_history]
    control_energies = [h['control_energy'] for h in energy_history]
    total_energies = [h['total_energy'] for h in energy_history]
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 7))
    fig.patch.set_facecolor('white')
    
    ax1.plot(times, lifting_energies, 'b-', linewidth=2)
    ax1.set_title('Lifting Energy vs Time', fontweight='bold')
    ax1.grid(True, alpha=0.3)
    
    ax2.plot(times, control_energies, 'r-', linewidth=2)
    ax2.set_title('Control Energy vs Time', fontweight='bold')
    ax2.grid(True, alpha=0.3)
    
    ax3.plot(times, total_energies, 'g-', linewidth=2)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Energy (J)')
    ax3.set_title('Total Energy Required vs Time', fontweight='bold')
    ax3.grid(True, alpha=0.3)
    
    if len(total_energies) > 0:
        final_energy = total_energies[-1]
        ax3.annotate(f'Final: {final_energy:.1f} J', xy=(times[-1], final_energy),
                     xytext=(times[-1]*0.7, final_energy), arrowprops=dict(arrowstyle='->', color='green'))
    
    plt.tight_layout()
    
    canvas = FigureCanvasAgg(fig)
    canvas.draw()
    buf = canvas.buffer_rgba()
    w, h = canvas.get_width_height()
    img_array = np.frombuffer(buf, dtype=np.uint8).reshape(h, w, 4)
    graph_surface = pygame.surfarray.make_surface(img_array[:, :, :3].swapaxes(0, 1))
    plt.close(fig)


def analyze_length_vs_energy():
    """
    Target Cable Length 1.0~20.0m 정밀 분석 후 데이터 출력
    """
    global length_analysis_data, length_analysis_surface
    
    print("\n[Analysis] Calculating Energy (Range: 1.0m ~ 20.0m)...")
    print("Skipping unstable region (0.1m ~ 1.0m) to fix graph scaling...")
    length_analysis_data = []
    
    # 1.0m 부터 시작 (스케일링 문제 해결)
    steps = np.arange(1.0, 20.05, 0.05)
    
    for i, target_L_float in enumerate(steps):
        lifting_e, control_e, total_e = run_trial_batch_detailed(
            target_L_float, 
            INITIAL_ACCEL, 
            ACCEL_DURATION, 
            TOTAL_TIME
        )
        
        length_analysis_data.append({
            'target_length': target_L_float,
            'lifting_energy': lifting_e,
            'control_energy': control_e,
            'total_energy': total_e
        })
        
        if i % 20 == 0:
            sys.stdout.write(f"\r Processing: {target_L_float:.2f}m / 20.00m")
            sys.stdout.flush()
    
    print("\nAnalysis complete!")
    print("\n" + "="*60)
    print(">>> Analysis Complete <<<")
    print("="*60 + "\n")
  
    create_length_analysis_graph()


def create_length_analysis_graph():
    """정밀 분석 결과 그래프 (NaN 안전장치 포함)"""
    global length_analysis_surface
    
    if not length_analysis_data: return
    
    lengths = [d['target_length'] for d in length_analysis_data]
    
    # numpy array로 변환
    total_energies = np.array([d['total_energy'] for d in length_analysis_data])
    lifting_energies = np.array([d['lifting_energy'] for d in length_analysis_data])
    control_energies = np.array([d['control_energy'] for d in length_analysis_data])
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 7))
    fig.patch.set_facecolor('white')
    
    def set_grid_style(ax):
        ax.minorticks_on()
        ax.grid(True, which='major', alpha=0.6, linestyle='-')
        ax.grid(True, which='minor', alpha=0.2, linestyle=':')
    
    # --- Graph 1: Total Energy ---
    ax1.plot(lengths, total_energies, 'b-', linewidth=2)
    ax1.set_xlabel('Target Cable Length (m)', fontsize=10, fontweight='bold')
    ax1.set_ylabel('Total Energy (J)', fontsize=10, fontweight='bold')
    ax1.set_title('Target Cable Length vs Total Energy Required', fontsize=12, fontweight='bold')
    ax1.set_xlim([0, 20.5])
    
    valid_totals = total_energies[~np.isnan(total_energies)]
    y_max_1 = np.max(valid_totals) if len(valid_totals) > 0 else 100.0
    ax1.set_ylim([0, y_max_1 * 1.1])
    set_grid_style(ax1)
    
    # --- Graph 2: Control Energy ---
    ax2.plot(lengths, control_energies, 'r-', linewidth=2)
    ax2.set_xlabel('Target Cable Length (m)', fontsize=10, fontweight='bold')
    ax2.set_ylabel('Control Energy (J)', fontsize=10, fontweight='bold')
    ax2.set_title('Target Cable Length vs Control Torque Energy', fontsize=12, fontweight='bold')
    ax2.set_xlim([0, 20.5])
    
    valid_controls = control_energies[~np.isnan(control_energies)]
    y_max_2 = np.max(valid_controls) if len(valid_controls) > 0 else 100.0
    ax2.set_ylim([0, y_max_2 * 1.2])
    set_grid_style(ax2)
    
    # --- Graph 3: Lifting Energy ---
    ax3.plot(lengths, lifting_energies, 'g-', linewidth=2)
    ax3.set_xlabel('Target Cable Length (m)', fontsize=10, fontweight='bold')
    ax3.set_ylabel('Lifting Energy (J)', fontsize=10, fontweight='bold')
    ax3.set_title('Target Cable Length vs Lifting Energy', fontsize=12, fontweight='bold')
    ax3.set_xlim([0, 20.5])
    
    valid_lifts = lifting_energies[~np.isnan(lifting_energies)]
    if len(valid_lifts) > 0:
        y_min_3 = np.min(valid_lifts)
        y_max_3 = np.max(valid_lifts)
        y_range_3 = y_max_3 - y_min_3 if y_max_3 != y_min_3 else 10.0
        ax3.set_ylim([y_min_3 - y_range_3 * 0.1, y_max_3 + y_range_3 * 0.1])
    
    set_grid_style(ax3)
    
    plt.tight_layout(rect=[0.02, 0.03, 0.98, 0.97])
    
    canvas = FigureCanvasAgg(fig)
    canvas.draw()
    buf = canvas.buffer_rgba()
    w, h = canvas.get_width_height()
    img_array = np.frombuffer(buf, dtype=np.uint8).reshape(h, w, 4)
    length_analysis_surface = pygame.surfarray.make_surface(img_array[:, :, :3].swapaxes(0, 1))
    plt.close(fig)


def calculate_drag_force(velocity, angle):
    if abs(velocity) < 0.01: return 0.0
    A_eff = A_frontal * abs(np.cos(angle)) + (CONTAINER_LENGTH * CONTAINER_HEIGHT) * abs(np.sin(angle))
    return -0.5 * rho_air * C_d * A_eff * velocity * abs(velocity)

def calculate_control_torque(pendulum_angle, pendulum_angular_velocity):
    return -Kp * pendulum_angle - Kd * pendulum_angular_velocity


# [수정 사항] 제어 에너지 계산 시 질량(m) 곱함
def run_trial_batch(target_L_value, a0, accel_duration, total_time):
    theta_local = 0.0
    omega_local = 0.0
    crane_v_local = 0.0
    crane_x_m_local = 0.0
    t = 0.0
    motion_t = 0.0 
    
    lifting_E = 0.0
    control_E = 0.0
    
    L_current = L_total
    is_lifting_local = True
    lifting_done_local = False
  
    while t < total_time:
        if is_lifting_local and not lifting_done_local:
            if L_current > target_L_value:
                delta_L = lifting_speed * dt
                if L_current - delta_L < target_L_value:
                    actual_delta = L_current - target_L_value
                    L_current = target_L_value
                    lifting_E += m * g * actual_delta
                    is_lifting_local = False
                    lifting_done_local = True
                else:
                    L_current -= delta_L
                    lifting_E += m * g * delta_L
            else:
                is_lifting_local = False
                lifting_done_local = True
        
        if lifting_done_local:
            container_v = crane_v_local + L_current * omega_local * np.cos(theta_local)
            drag = calculate_drag_force(container_v, theta_local)
            
            if motion_t < accel_duration:
                crane_a = a0 + drag / m
                crane_v_new = crane_v_local + crane_a * dt
                crane_x_new = crane_x_m_local + crane_v_new * dt
            else:
                crane_a = 0.0
                crane_v_new = 0.0
                crane_x_new = crane_x_m_local
                
            ctrl_torque = calculate_control_torque(theta_local, omega_local)
            alpha = -(g/L_current)*np.sin(theta_local) - (crane_a/L_current)*np.cos(theta_local) + ctrl_torque/L_current
            
            omega_new = omega_local + alpha * dt
            theta_new = theta_local + omega_new * dt
            
            # [수정] Energy = Torque * Omega (여기서 Torque는 실제 N·m 단위가 되도록 m 곱함)
            # ctrl_torque는 시뮬레이션 상 단위 질량당 가속 기여도로 계산되고 있었음.
            power = (ctrl_torque * m) * omega_new 
            control_E += abs(power) * dt
            
            theta_local = theta_new
            omega_local = omega_new
            crane_v_local = crane_v_new
            crane_x_m_local = crane_x_new
            
            motion_t += dt
            
        t += dt
        
    return lifting_E + control_E


# [수정 사항] 제어 에너지 계산 시 질량(m) 곱함
def run_trial_batch_detailed(target_L_value, a0, accel_duration, total_time):
    theta_local = 0.0
    omega_local = 0.0
    crane_v_local = 0.0
    crane_x_m_local = 0.0
    t = 0.0
    motion_t = 0.0 
    
    lifting_E = 0.0
    control_E = 0.0
    
    L_current = L_total
    is_lifting_local = True
    lifting_done_local = False
  
    while t < total_time:
        if is_lifting_local and not lifting_done_local:
            if L_current > target_L_value:
                delta_L = lifting_speed * dt
                if L_current - delta_L < target_L_value:
                    actual_delta = L_current - target_L_value
                    L_current = target_L_value
                    lifting_E += m * g * actual_delta
                    is_lifting_local = False
                    lifting_done_local = True
                else:
                    L_current -= delta_L
                    lifting_E += m * g * delta_L
            else:
                is_lifting_local = False
                lifting_done_local = True
        
        if lifting_done_local:
            container_v = crane_v_local + L_current * omega_local * np.cos(theta_local)
            drag = calculate_drag_force(container_v, theta_local)
            
            if motion_t < accel_duration:
                crane_a = a0 + drag / m
                crane_v_new = crane_v_local + crane_a * dt
                crane_x_new = crane_x_m_local + crane_v_new * dt
            else:
                crane_a = 0.0
                crane_v_new = 0.0
                crane_x_new = crane_x_m_local
                
            ctrl_torque = calculate_control_torque(theta_local, omega_local)
            alpha = -(g/L_current)*np.sin(theta_local) - (crane_a/L_current)*np.cos(theta_local) + ctrl_torque/L_current
            
            omega_new = omega_local + alpha * dt
            theta_new = theta_local + omega_new * dt
            
            # [수정] 질량 곱하기 적용
            power = (ctrl_torque * m) * omega_new
            control_E += abs(power) * dt
            
            theta_local = theta_new
            omega_local = omega_new
            crane_v_local = crane_v_new
            crane_x_m_local = crane_x_new
            
            motion_t += dt
            
        t += dt
        
    return lifting_E, control_E, lifting_E + control_E


def find_optimal_accel(L_value, disp_limit=DISP_LIMIT, accel_duration=ACCEL_DURATION, total_time=TOTAL_TIME, a_min=A_MIN, a_max=A_MAX, da=DA):
    last_safe = a_min
    a0 = a_min
    while a0 <= a_max:
        theta_local = 0.0
        omega_local = 0.0
        crane_v_local = 0.0
        crane_x_m_local = 0.0
        t = 0.0
        max_disp = 0.0
        L_current = L_value
        
        while t < total_time:
            container_v = crane_v_local + L_current * omega_local * np.cos(theta_local)
            drag = calculate_drag_force(container_v, theta_local)
            if t < accel_duration:
                crane_a = a0 + drag/m
                crane_v_new = crane_v_local + crane_a * dt
                crane_x_new = crane_x_m_local + crane_v_new * dt
            else:
                crane_a = 0.0
                crane_v_new = 0.0
                crane_x_new = crane_x_m_local
                
            ctrl = calculate_control_torque(theta_local, omega_local)
            alpha = -(g/L_current)*np.sin(theta_local) - (crane_a/L_current)*np.cos(theta_local) + ctrl/L_current
            omega_new = omega_local + alpha * dt
            theta_new = theta_local + omega_new * dt
            
            if abs(L_current * np.sin(theta_new)) > max_disp:
                max_disp = abs(L_current * np.sin(theta_new))
            
            theta_local = theta_new
            omega_local = omega_new
            crane_v_local = crane_v_new
            crane_x_m_local = crane_x_new
            t += dt
            
        if max_disp > disp_limit: break
        last_safe = a0
        a0 += da
    return last_safe

def update_physics():
    global crane_x, crane_v, theta, omega, time_elapsed
    global max_angle_deg, max_horizontal_disp, max_tension, max_control_torque
    global control_energy_net, control_energy_pos, control_energy_neg
    global cumulative_kinetic_energy, L, is_lifting, lifting_completed
    global lifting_energy, total_energy_required, lifting_msg_shown
    
    # 인양 (Lifting) - target_cable_length 기준
    if is_lifting and not lifting_completed:
        if L > target_cable_length:
            delta_L = lifting_speed * dt
            if L - delta_L < target_cable_length:
                actual_delta = L - target_cable_length
                L = target_cable_length
                lifting_energy += m * g * actual_delta
                is_lifting = False
                lifting_completed = True
                lifting_msg_shown = False
            else:
                L -= delta_L
                lifting_energy += m * g * delta_L
        else:
            is_lifting = False
            lifting_completed = True
            lifting_msg_shown = False
  
    crane_x_m = (crane_x - WIDTH // 2) / SCALE
    container_v = crane_v + L * omega * np.cos(theta)
    
    drag_force = calculate_drag_force(container_v, theta)
    control_torque = calculate_control_torque(theta, omega)
  
    if lifting_completed:
        if trial_phase == "hold":
            crane_a = 0.0
            crane_v_new = crane_v
            crane_x_m_new = crane_x_m
        else:
            crane_a = external_accel + drag_force / m
            crane_v_new = crane_v + crane_a * dt
            crane_x_m_new = crane_x_m + crane_v_new * dt
    else:
        crane_a = 0.0
        crane_v_new = 0.0
        crane_x_m_new = crane_x_m
  
    if lifting_completed:
        # [수정] 실시간 계산에서도 질량(m) 적용
        power_control = (control_torque * m) * omega
        dE = power_control * dt
        control_energy_net += dE
        if dE > 0: control_energy_pos += dE
        else: control_energy_neg += dE
  
    total_energy_required = lifting_energy + abs(control_energy_net)
  
    rot_KE = 0.5 * m * L * L * omega * omega
    trans_KE = 0.5 * m * container_v * container_v
    total_KE = rot_KE + trans_KE
    cumulative_kinetic_energy += total_KE * dt
  
    force_history.append({
        'time': time_elapsed,
        'drag': drag_force,
        'control': control_torque,
        'total': drag_force,
        'velocity': crane_v,
        'effective_area': A_frontal * abs(np.cos(theta)) + (CONTAINER_LENGTH * CONTAINER_HEIGHT) * abs(np.sin(theta)),
        'kinetic_energy': total_KE,
        'cumulative_kinetic_energy': cumulative_kinetic_energy
    })
    angle_history.append({'time': time_elapsed, 'angle': float(np.degrees(theta)), 'omega': omega})
    energy_history.append({'time': time_elapsed, 'lifting_energy': lifting_energy, 'control_energy': abs(control_energy_net), 'total_energy': total_energy_required})
  
    if len(force_history) > 1000: force_history.pop(0)
    if len(angle_history) > 1000: angle_history.pop(0)
    if len(energy_history) > 1000: energy_history.pop(0)
  
    alpha = -(g / L) * np.sin(theta) - (crane_a / L) * np.cos(theta) + control_torque / L
    tension = m * (g * np.cos(theta) + L * omega ** 2 + crane_a * np.sin(theta))
  
    omega_new = omega + alpha * dt
    theta_new = theta + omega_new * dt
  
    omega = omega_new
    theta = theta_new
    crane_v = crane_v_new
    crane_x = int(crane_x_m_new * SCALE + WIDTH // 2)
    crane_x = max(100, min(WIDTH - 100, crane_x))
  
    if trial_running:
        max_angle_deg = max(max_angle_deg, abs(float(np.degrees(theta))))
        max_horizontal_disp = max(max_horizontal_disp, abs(L * np.sin(theta)))
        max_tension = max(max_tension, float(tension))
        max_control_torque = max(max_control_torque, abs(float(control_torque)))
  
    time_elapsed += dt

def draw_container(x, y, angle):
    w, h = CONTAINER_PIXEL_WIDTH, CONTAINER_PIXEL_HEIGHT
    cos_a, sin_a = np.cos(angle), np.sin(angle)
    corners = [(-w/2, -h/2), (w/2, -h/2), (w/2, h/2), (-w/2, h/2)]
    rotated = []
    for cx, cy in corners:
        rx = cx * cos_a - cy * sin_a
        ry = cx * sin_a + cy * cos_a
        rotated.append((x + rx, y + ry))
    pygame.draw.polygon(screen, ORANGE, rotated)
    pygame.draw.polygon(screen, DARK_GRAY, rotated, 3)

def draw():
    global length_rect, height_rect
    screen.fill(WHITE)
    
    crane_top = 100
    pygame.draw.rect(screen, GRAY, (crane_x - 30, crane_top - 20, 60, 20))
    pygame.draw.line(screen, GRAY, (crane_x, crane_top), (crane_x, 50), 5)
  
    container_x = crane_x + int(L * SCALE * np.sin(theta))
    container_y = crane_top + int(L * SCALE * np.cos(theta))
  
    rope_offset = 20
    pygame.draw.line(screen, BLACK, (crane_x - rope_offset, crane_top), (container_x - rope_offset//2, container_y - 40//2), 2)
    pygame.draw.line(screen, BLACK, (crane_x + rope_offset, crane_top), (container_x + rope_offset//2, container_y - 40//2), 2)
  
    draw_container(container_x, container_y, theta)
    
    target_y = crane_top + int((L_total - target_height) * SCALE)
    pygame.draw.line(screen, (255, 100, 100), (0, target_y), (WIDTH, target_y), 2)
    screen.blit(font.render(f"Target Height: {target_height:.1f}m", True, (255, 100, 100)), (WIDTH - 200, target_y - 20))
  
    # 정보 텍스트
    y_off = 20
    screen.blit(large_font.render("Port Crane Energy Analysis (Synced Input)", True, BLACK), (20, y_off))
    y_off += 50
    screen.blit(font.render("T=Trigger | O=Opt | G=Graph | L=Len Analysis | R=Reset", True, BLUE), (20, y_off))
    y_off += 30
    
    params = [
        f"Total Cable: {L_total:.1f}m",
        f"Current Length: {L:.2f}m",
        f"Target Cable Length: {target_cable_length:.2f}m",
        f"Target Height: {target_height:.2f}m",
        f"Lift Status: {'LIFTING' if is_lifting else 'DONE'}",
        f"Mass: {m}kg, a0: {INITIAL_ACCEL}m/s2",
    ]
    for p in params:
        screen.blit(font.render(p, True, BLACK), (20, y_off))
        y_off += 25
        
    # 결과 박스
    if has_results:
        pygame.draw.rect(screen, (240, 255, 240), (WIDTH-350, HEIGHT-180, 330, 160))
        pygame.draw.rect(screen, GREEN, (WIDTH-350, HEIGHT-180, 330, 160), 2)
        res_strs = [
            "Trial Results:",
            f"Max Swing: {max_angle_deg:.2f} deg",
            f"Lifting E: {lifting_energy:.1f} J",
            f"Control E: {abs(control_energy_net):.1f} J",
            f"Total E: {total_energy_required:.1f} J"
        ]
        for i, s in enumerate(res_strs):
            screen.blit(font.render(s, True, BLACK), (WIDTH-340, HEIGHT-170 + i*25))
  
    # 그래프 1: Angle (썸네일)
    gx, gy, gw, gh = WIDTH-420, 50, 400, 150
    pygame.draw.rect(screen, (250,250,250), (gx, gy, gw, gh))
    pygame.draw.rect(screen, BLACK, (gx, gy, gw, gh), 1)
    if len(angle_history) > 1:
        pts = []
        for i in range(len(angle_history)):
            px = gx + (i/len(angle_history))*gw
            py = gy + gh/2 - (angle_history[i]['angle']/10.0)*(gh/2)
            pts.append((px, py))
        if len(pts)>1: pygame.draw.lines(screen, RED, False, pts, 2)
    screen.blit(font.render("Angle vs Time", True, BLACK), (gx+5, gy+5))
  
    # 그래프 2: Energy (썸네일)
    gy2 = gy + gh + 20
    pygame.draw.rect(screen, (250,250,250), (gx, gy2, gw, gh))
    pygame.draw.rect(screen, BLACK, (gx, gy2, gw, gh), 1)
    
    if len(energy_history) > 1:
        max_e = max(h['total_energy'] for h in energy_history) + 1.0
        pts2 = []
        for i in range(len(energy_history)):
            px = gx + (i/len(energy_history))*gw
            val = energy_history[i]['total_energy']
            py = gy2 + gh - 10 - (val/max_e)*(gh-20)
            pts2.append((px, py))
        if len(pts2)>1: pygame.draw.lines(screen, PURPLE, False, pts2, 2)
        
        curr_e = energy_history[-1]['total_energy']
        screen.blit(font.render(f"{curr_e:.1f} J", True, PURPLE), (gx+gw-80, gy2+5))
        
    screen.blit(font.render("Total Energy vs Time", True, BLACK), (gx+5, gy2+5))
  
    # 입력창
    box_w, box_h = 200, 32
    bx, by = WIDTH//2 - 220, HEIGHT - 60
    
    # 입력창 1: Target Cable Length
    screen.blit(font.render("Target Cable Length (m):", True, BLACK), (bx, by-25))
    pygame.draw.rect(screen, BLUE if length_input_active else (230,230,230), (bx, by, box_w, box_h), 2)
    disp_L = input_length_text if input_length_text else f"{target_cable_length:.2f}"
    screen.blit(font.render(disp_L, True, BLACK), (bx+5, by+5))
    length_rect = pygame.Rect(bx, by, box_w, box_h)
    
    # 입력창 2: Target Height
    bx2 = WIDTH//2 + 20
    screen.blit(font.render("Target Height (m):", True, BLACK), (bx2, by-25))
    pygame.draw.rect(screen, BLUE if height_input_active else (230,230,230), (bx2, by, box_w, box_h), 2)
    disp_H = input_height_text if input_height_text else f"{target_height:.2f}"
    screen.blit(font.render(disp_H, True, BLACK), (bx2+5, by+5))
    height_rect = pygame.Rect(bx2, by, box_w, box_h)
  
    # 팝업 그래프
    if show_matplotlib_graph and graph_surface:
        screen.blit(graph_surface, ((WIDTH-graph_surface.get_width())//2, (HEIGHT-graph_surface.get_height())//2))
    if show_length_analysis_graph and length_analysis_surface:
        screen.blit(length_analysis_surface, ((WIDTH-length_analysis_surface.get_width())//2, (HEIGHT-length_analysis_surface.get_height())//2))
  
    pygame.display.flip()

def reset_simulation():
    global crane_x, crane_v, theta, omega, time_elapsed, force_history, angle_history, energy_history
    global trial_running, trial_phase, trial_time, external_accel
    global max_angle_deg, max_horizontal_disp, max_tension, max_control_torque, has_results
    global control_energy_net, control_energy_pos, control_energy_neg, cumulative_kinetic_energy
    global is_lifting, lifting_completed, lifting_energy, total_energy_required, lifting_msg_shown, L
    
    crane_x, crane_v, theta, omega = WIDTH//2, 0.0, 0.0, 0.0
    time_elapsed = 0.0
    force_history, angle_history, energy_history = [], [], []
    trial_running, trial_phase, trial_time, external_accel = False, "idle", 0.0, 0.0
    L, is_lifting, lifting_completed = L_total, True, False
    max_angle_deg, max_horizontal_disp, max_tension, max_control_torque, has_results = 0,0,0,0,False
    control_energy_net, control_energy_pos, control_energy_neg = 0,0,0
    cumulative_kinetic_energy, lifting_energy, total_energy_required, lifting_msg_shown = 0,0,0,False

# Main Loop
running = True
print("[System] Simulation Started. Input Target Cable Length to auto-calc Target Height.")
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT: running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE: running = False
            elif event.key == pygame.K_r: reset_simulation()
            elif event.key == pygame.K_t:
                if lifting_completed:
                    trial_running, trial_phase, trial_time, has_results = True, "accelerate", 0.0, True
            elif event.key == pygame.K_o:
                if lifting_completed:
                    a_safe = find_optimal_accel(target_cable_length)
                    print(f"Optimal a0: {a_safe}")
            elif event.key == pygame.K_g: show_matplotlib_graph = not show_matplotlib_graph
            elif event.key == pygame.K_l:
                if show_length_analysis_graph: show_length_analysis_graph = False
                else:
                    analyze_length_vs_energy()
                    show_length_analysis_graph = True
            
            elif event.key == pygame.K_RETURN:
                if length_input_active and input_length_text.strip():
                    try:
                        val = float(input_length_text)
                        if 0.1 <= val <= L_total:
                            target_cable_length = val
                            target_height = L_total - target_cable_length
                            reset_simulation()
                    except: pass
                    input_length_text = ""
                
                if height_input_active and input_height_text.strip():
                    try:
                        val = float(input_height_text)
                        if 0 <= val <= L_total:
                            target_height = val
                            target_cable_length = L_total - target_height
                            reset_simulation()
                    except: pass
                    input_height_text = ""
            
            elif event.key == pygame.K_BACKSPACE:
                if length_input_active: input_length_text = input_length_text[:-1]
                if height_input_active: input_height_text = input_height_text[:-1]
            else:
                if length_input_active and event.unicode in "0123456789.-": input_length_text += event.unicode
                if height_input_active and event.unicode in "0123456789.-": input_height_text += event.unicode
        
        elif event.type == pygame.MOUSEBUTTONDOWN:
            mx, my = event.pos
            if length_rect and length_rect.collidepoint(mx, my):
                length_input_active, height_input_active = True, False
            elif height_rect and height_rect.collidepoint(mx, my):
                length_input_active, height_input_active = False, True
            else:
                length_input_active, height_input_active = False, False
  
    if trial_running and lifting_completed:
        trial_time += dt
        if trial_phase == "accelerate":
            external_accel = INITIAL_ACCEL
            if trial_time >= ACCEL_DURATION:
                trial_phase, external_accel, crane_v = "hold", 0.0, 0.0
        elif trial_phase == "hold": external_accel = 0.0
    else: external_accel = 0.0
    
    if lifting_completed and not lifting_msg_shown:
        print(f"Lifting Done. L={L:.2f}m")
        lifting_msg_shown = True
  
    update_physics()
    draw()
    clock.tick(50)
  
pygame.quit()
sys.exit()