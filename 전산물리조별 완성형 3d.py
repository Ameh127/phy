import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import sys
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg

# ===============================
# 창 / 기본 상수
# ===============================
WIDTH, HEIGHT = 1280, 720

# 물리 상수
g = 9.81          # 중력 (m/s^2)
L_total = 20.0    # 케이블 총 길이 (m)
m = 15000.0       # 컨테이너 질량 (kg)

# 컨테이너 실제 치수 (m)
CONTAINER_LENGTH = 12.192
CONTAINER_WIDTH  = 2.438
CONTAINER_HEIGHT = 2.591

# 시뮬레이션 세계 좌표에서의 컨테이너 스케일
SCALE = 0.15

# 공기저항 관련
rho_air = 1.225
C_d = 1.05
A_frontal = CONTAINER_WIDTH * CONTAINER_HEIGHT

# 이동 프로파일 파라미터
INITIAL_ACCEL = 1.5     # 가속도
ACCEL_DURATION = 1.0    # 가속 시간
COAST_DURATION = 3.0    # 등속 이동 시간
DECEL_DURATION = 1.0    # 감속 시간
TOTAL_SIM_TIME = 15.0   # 분석용 총 시뮬레이션 시간

# 제어 파라미터
Kp = 27.0
Kd = 18.0

# 목표 케이블 길이 설정
target_cable_length = 3.0
target_height = L_total - target_cable_length

# 인양 속도
lifting_speed = 2.0

# 크레인 상태
crane_x = 0.0
crane_v = 0.0
external_accel = 0.0

# 진자 상태
theta = 0.0
omega = 0.0

# 케이블 길이
L = L_total
is_lifting = True
lifting_completed = False

# 시뮬레이션 시간
dt = 0.02
time_elapsed = 0.0

# 실험 상태
trial_running = False
trial_phase = "idle"
trial_time = 0.0

# 결과값
max_angle_deg = 0.0
max_horizontal_disp = 0.0
has_results = False

# 에너지
lifting_energy = 0.0
control_energy_net = 0.0
total_energy_required = 0.0

# 에너지 이력 저장용
energy_history = [] 

# 카메라
cam_yaw = math.radians(40)
cam_pitch = -0.4
cam_dist = 35.0

# 그래프 텍스처용 변수
graph_surface = None
graph_texture_id = None

# [추가] 길이 분석용 변수
show_length_analysis_graph = False
length_analysis_texture_id = None
length_analysis_data = []

def set_camera():
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    cx, cy, cz = 0.0, -12.0, 0.0
    ex = cx + cam_dist * math.cos(cam_pitch) * math.sin(cam_yaw)
    ey = cy + cam_dist * math.sin(cam_pitch)
    ez = cz + cam_dist * math.cos(cam_pitch) * math.cos(cam_yaw)
    gluLookAt(ex, ey, ez, cx, cy, cz, 0.0, 1.0, 0.0)

# HUD 설정
HUD_RATIO = 0.28
HUD_W = int(WIDTH * HUD_RATIO)
HUD_H = HEIGHT

hud_surface = None
hud_texture_id = None
font = None
large_font = None

input_length_text = ""
input_height_text = ""
active_field = "length"

length_rect = None
height_rect = None

def init_hud():
    global hud_surface, hud_texture_id, font, large_font
    global graph_texture_id, length_analysis_texture_id
    
    pygame.font.init()
    try:
        font = pygame.font.SysFont("Arial", 15)
        large_font = pygame.font.SysFont("Arial", 18, bold=True)
    except:
        font = pygame.font.Font(None, 20)
        large_font = pygame.font.Font(None, 24)
    
    hud_surface = pygame.Surface((HUD_W, HUD_H), pygame.SRCALPHA, 32).convert_alpha()
    
    # 텍스처 생성 도우미 함수
    def gen_tex():
        tid = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, tid)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        return tid

    hud_texture_id = gen_tex()
    graph_texture_id = gen_tex()
    length_analysis_texture_id = gen_tex() # [추가] 분석 그래프용 텍스처
    
    glBindTexture(GL_TEXTURE_2D, 0)

def update_graph_texture():
    global graph_surface
    if len(energy_history) < 2: return

    display_data = energy_history[-500:]
    times = [d['time'] for d in display_data]
    energies = [d['total_energy'] for d in display_data]

    fig, ax = plt.subplots(figsize=(4, 3), dpi=80)
    fig.patch.set_alpha(0.0)
    ax.patch.set_alpha(0.5)
    ax.set_facecolor('white')
    
    ax.plot(times, energies, 'g-', linewidth=2)
    ax.set_title("Total Energy vs Time", fontsize=10, color='white', fontweight='bold')
    ax.set_xlabel("Time (s)", fontsize=8, color='white')
    ax.set_ylabel("Energy (J)", fontsize=8, color='white')
    ax.tick_params(colors='white', labelsize=8)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()

    canvas = FigureCanvasAgg(fig)
    canvas.draw()
    raw_data = canvas.buffer_rgba()
    w, h = canvas.get_width_height()
    plt.close(fig)

    graph_surface = pygame.image.frombuffer(raw_data, (w, h), "RGBA")

    glBindTexture(GL_TEXTURE_2D, graph_texture_id)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, raw_data)
    glBindTexture(GL_TEXTURE_2D, 0)

# ===============================
# [추가] 분석 모드 관련 함수들
# ===============================
def calculate_drag_force(velocity, angle):
    if abs(velocity) < 0.01: return 0.0
    A_eff = A_frontal * abs(np.cos(angle)) + (CONTAINER_LENGTH * CONTAINER_HEIGHT) * abs(np.sin(angle))
    return -0.5 * rho_air * C_d * A_eff * velocity * abs(velocity)

def calculate_control_torque(pendulum_angle, pendulum_angular_velocity):
    return -Kp * pendulum_angle - Kd * pendulum_angular_velocity

# [추가] 배치 시뮬레이션 (화면 렌더링 없이 물리만 계산)
def run_trial_batch_detailed(target_L_value):
    # 로컬 변수로 시뮬레이션 수행 (글로벌 상태 영향 X)
    theta_loc = 0.0
    omega_loc = 0.0
    crane_v_loc = 0.0
    crane_x_loc = 0.0
    
    t = 0.0
    motion_t = 0.0
    
    lifting_E = 0.0
    control_E = 0.0
    
    L_current = L_total
    is_lifting_loc = True
    lifting_done_loc = False
    
    # 리프팅 + 이동 시뮬레이션
    while t < TOTAL_SIM_TIME:
        # 1. 리프팅 로직
        if is_lifting_loc and not lifting_done_loc:
            if L_current > target_L_value:
                delta_L = lifting_speed * dt
                if L_current - delta_L < target_L_value:
                    actual_delta = L_current - target_L_value
                    L_current = target_L_value
                    lifting_E += m * g * actual_delta
                    is_lifting_loc = False
                    lifting_done_loc = True
                else:
                    L_current -= delta_L
                    lifting_E += m * g * delta_L
            else:
                is_lifting_loc = False
                lifting_done_local = True

        # 2. 이동 로직 (Accel -> Coast -> Decel -> Hold)
        crane_a = 0.0
        if lifting_done_loc:
            container_v = crane_v_loc + L_current * omega_loc * np.cos(theta_loc)
            drag = calculate_drag_force(container_v, theta_loc)
            
            ext_accel = 0.0
            if motion_t < ACCEL_DURATION:
                ext_accel = INITIAL_ACCEL
            elif motion_t < ACCEL_DURATION + COAST_DURATION:
                ext_accel = 0.0
            elif motion_t < ACCEL_DURATION + COAST_DURATION + DECEL_DURATION:
                ext_accel = -INITIAL_ACCEL
                if crane_v_loc <= 0: ext_accel = 0.0 # 정지 보정
            else:
                ext_accel = 0.0
            
            # 감속 단계에서 속도가 0 이하면 강제 정지
            if motion_t > ACCEL_DURATION + COAST_DURATION and crane_v_loc <= 0.0 and ext_accel <= 0:
                ext_accel = 0.0
                crane_v_loc = 0.0
                crane_a = 0.0
            else:
                crane_a = ext_accel + drag / m

            crane_v_new = crane_v_loc + crane_a * dt
            crane_x_new = crane_x_loc + crane_v_new * dt
            
            # 물리 업데이트
            ctrl_torque = calculate_control_torque(theta_loc, omega_loc)
            alpha = -(g/L_current)*np.sin(theta_loc) - (crane_a/L_current)*np.cos(theta_loc) + ctrl_torque/L_current
            
            omega_new = omega_loc + alpha * dt
            theta_new = theta_loc + omega_new * dt
            
            # [중요] 제어 에너지 계산 (질량 m 포함)
            power = (ctrl_torque * m) * omega_new
            control_E += abs(power) * dt
            
            theta_loc = theta_new
            omega_loc = omega_new
            crane_v_loc = crane_v_new
            crane_x_loc = crane_x_new
            
            motion_t += dt
            
        t += dt
        
    return lifting_E, control_E, lifting_E + control_E

# [추가] 분석 실행 및 그래프 생성
def analyze_length_vs_energy():
    global length_analysis_data
    print("------------------------------------------------")
    print("[Analysis] Calculating Energy for lengths 1.0m ~ 20.0m...")
    print("This may take a moment...")
    
    length_analysis_data = []
    # 1.0m 부터 20.0m 까지 0.5m 간격으로 분석
    steps = np.arange(1.0, 20.1, 0.5)
    
    for val in steps:
        l_e, c_e, t_e = run_trial_batch_detailed(val)
        length_analysis_data.append({
            'len': val, 'lift': l_e, 'ctrl': c_e, 'total': t_e
        })
    
    print("[Analysis] Complete.")
    print("------------------------------------------------")
    create_length_analysis_graph()

# [추가] 분석 그래프 그리기 (Matplotlib -> OpenGL Texture)
def create_length_analysis_graph():
    if not length_analysis_data: return
    
    lens = [d['len'] for d in length_analysis_data]
    total_e = np.array([d['total'] for d in length_analysis_data])
    ctrl_e = np.array([d['ctrl'] for d in length_analysis_data])
    lift_e = np.array([d['lift'] for d in length_analysis_data])
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), dpi=100)
    fig.patch.set_facecolor('white')
    
    # 스타일 설정 함수
    def style_ax(ax, title, y_label, color):
        ax.set_title(title, fontweight='bold', fontsize=10)
        ax.set_ylabel(y_label, fontsize=9)
        ax.grid(True, alpha=0.5)
        ax.tick_params(labelsize=8)

    # 1. Total Energy
    ax1.plot(lens, total_e, 'b.-')
    style_ax(ax1, "Total Energy Required (J)", "Energy (J)", 'b')
    
    # 2. Control Energy
    ax2.plot(lens, ctrl_e, 'r.-')
    style_ax(ax2, "Control Energy (Stabilization) (J)", "Energy (J)", 'r')
    
    # 3. Lifting Energy
    ax3.plot(lens, lift_e, 'g.-')
    style_ax(ax3, "Lifting Energy (J)", "Energy (J)", 'g')
    ax3.set_xlabel("Cable Length (m)", fontsize=10)
    
    plt.tight_layout()
    
    canvas = FigureCanvasAgg(fig)
    canvas.draw()
    raw_data = canvas.buffer_rgba()
    w, h = canvas.get_width_height()
    plt.close(fig)
    
    glBindTexture(GL_TEXTURE_2D, length_analysis_texture_id)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, raw_data)
    glBindTexture(GL_TEXTURE_2D, 0)

# ===============================
# 메인 물리 루프
# ===============================
def update_physics():
    global crane_x, crane_v, theta, omega, time_elapsed
    global max_angle_deg, max_horizontal_disp
    global control_energy_net, L, is_lifting, lifting_completed
    global lifting_energy, total_energy_required, energy_history
    
    if is_lifting and not lifting_completed:
        if L > target_cable_length:
            delta_L = lifting_speed * dt
            if L - delta_L < target_cable_length:
                actual_delta = L - target_cable_length
                L = target_cable_length
                lifting_energy += m * g * actual_delta
                is_lifting = False
                lifting_completed = True
            else:
                L -= delta_L
                lifting_energy += m * g * delta_L
        else:
            is_lifting = False
            lifting_completed = True

    crane_x_m = crane_x
    container_v = crane_v + L * omega * np.cos(theta)
    
    drag_force = calculate_drag_force(container_v, theta)
    control_torque = calculate_control_torque(theta, omega)

    if lifting_completed:
        if trial_phase == "hold":
            crane_a = 0.0
            crane_v_new = 0.0
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
        power_control = (control_torque * m) * omega
        dE = power_control * dt
        control_energy_net += dE

    total_energy_required = lifting_energy + abs(control_energy_net)

    if int(time_elapsed * 100) % 5 == 0:  
        energy_history.append({'time': time_elapsed, 'total_energy': total_energy_required})

    alpha = -(g / L) * np.sin(theta) - (crane_a / L) * np.cos(theta) + control_torque / L

    omega_new = omega + alpha * dt
    theta_new = theta + omega_new * dt

    omega = omega_new
    theta = theta_new
    crane_v = crane_v_new
    crane_x = crane_x_m_new

    if trial_running:
        max_angle_deg = max(max_angle_deg, abs(float(np.degrees(theta))))
        max_horizontal_disp = max(max_horizontal_disp, abs(L * np.sin(theta)))

    time_elapsed += dt

# ===============================
# 3D 그리기
# ===============================
def draw_box(cx, cy, cz, sx, sy, sz, color):
    r, g_c, b = color
    glColor3f(r, g_c, b)
    x, y, z = sx / 2.0, sy / 2.0, sz / 2.0
    
    vertices = [
        [cx-x,cy-y,cz-z], [cx+x,cy-y,cz-z],
        [cx+x,cy+y,cz-z], [cx-x,cy+y,cz-z],
        [cx-x,cy-y,cz+z], [cx+x,cy-y,cz+z],
        [cx+x,cy+y,cz+z], [cx-x,cy+y,cz+z]
    ]
    
    faces = [
        [0,1,2,3], [4,5,6,7], [0,1,5,4],
        [3,2,6,7], [1,2,6,5], [0,3,7,4]
    ]

    glBegin(GL_QUADS)
    for face in faces:
        for idx in face:
            glVertex3fv(vertices[idx])
    glEnd()

    glColor3f(0.0, 0.0, 0.0)
    edges = [
        [0,1],[1,2],[2,3],[3,0],
        [4,5],[5,6],[6,7],[7,4],
        [0,4],[1,5],[2,6],[3,7]
    ]
    glBegin(GL_LINES)
    for e in edges:
        for idx in e:
            glVertex3fv(vertices[idx])
    glEnd()

def draw_ground():
    cont_visual_height = CONTAINER_HEIGHT * SCALE
    trolley_offset = 0.3
    ground_y = -(L_total + cont_visual_height / 2.0 + trolley_offset)
    draw_box(0.0, ground_y, 0.0, 100.0, 0.3, 30.0, (0.85, 0.85, 0.85))

def draw_scene():
    draw_ground()
    beam_y = 0.0
    beam_len = 30.0
    draw_box(crane_x, beam_y, 0.0, beam_len, 0.3, 0.8, (0.5, 0.5, 0.5))
    trolley_y = beam_y - 0.3
    draw_box(crane_x, trolley_y, 0.0, 0.8, 0.4, 0.6, (0.3, 0.3, 0.3))

    cont_sx = CONTAINER_LENGTH * SCALE
    cont_sy = CONTAINER_HEIGHT * SCALE
    cont_sz = CONTAINER_WIDTH * SCALE
    spreader_h = 0.15 

    dist_cm_to_top = (cont_sy / 2.0) + (spreader_h / 2.0)
    visual_rope_len = L - dist_cm_to_top
    rope_origin = np.array([crane_x, trolley_y, 0.0])
    
    cm_x = crane_x + L * np.sin(theta)
    cm_y = trolley_y - L * np.cos(theta)
    cm_z = 0.0
    
    attach_x = crane_x + visual_rope_len * np.sin(theta)
    attach_y = trolley_y - visual_rope_len * np.cos(theta)
    attach_z = 0.0

    target_y_world = trolley_y - (L_total - target_height)
    glColor3f(1.0, 0.4, 0.4)
    glLineWidth(2.0)
    glBegin(GL_LINES)
    glVertex3f(-40.0, target_y_world, 0.0)
    glVertex3f(40.0, target_y_world, 0.0)
    glEnd()

    glColor3f(0.1, 0.1, 0.1)
    glLineWidth(2.0)
    offset_top = 0.6 
    offset_bottom = 0.3 

    glBegin(GL_LINES)
    glVertex3f(rope_origin[0] - offset_top/2, rope_origin[1], rope_origin[2])
    glVertex3f(attach_x - offset_bottom/2, attach_y, attach_z)
    glVertex3f(rope_origin[0] + offset_top/2, rope_origin[1], rope_origin[2])
    glVertex3f(attach_x + offset_bottom/2, attach_y, attach_z)
    glEnd()

    glPushMatrix()
    glTranslatef(cm_x, cm_y, cm_z)
    glRotatef(math.degrees(theta), 0, 0, 1)

    draw_box(0.0, (cont_sy/2.0) + (spreader_h/2.0), 0.0, 
             cont_sx, spreader_h, cont_sz, (0.4, 0.4, 0.45))
    draw_box(0.0, 0.0, 0.0, cont_sx, cont_sy, cont_sz, (1.0, 0.55, 0.0))
    
    glColor3f(0.8, 0.4, 0.0)
    glLineWidth(1.0)
    glBegin(GL_LINES)
    steps = 10
    for i in range(1, steps):
        x_pos = -cont_sx/2 + (cont_sx/steps)*i
        glVertex3f(x_pos, -cont_sy/2, cont_sz/2 + 0.01)
        glVertex3f(x_pos, cont_sy/2, cont_sz/2 + 0.01)
    glEnd()
    glPopMatrix()

# ===============================
# HUD 및 UI
# ===============================
def update_hud_surface():
    global length_rect, height_rect
    hud_surface.fill((0,0,0,0))
    pygame.draw.rect(hud_surface, (240,245,255,230), (0,0,HUD_W,HUD_H))
    pygame.draw.rect(hud_surface, (0,0,0,255), (0,0,HUD_W,HUD_H), 2)
    
    m, x0, y = 10, 10, 10
    title = large_font.render("3D Port Crane Energy Analysis", True, (0,0,0))
    hud_surface.blit(title, (x0, y))
    y += 35
    
    status_str = f"{trial_phase.upper()}" if trial_running else "IDLE"
    color = (255,0,0) if trial_phase == "accelerate" else ((0,0,255) if trial_phase=="coast" else (0,0,0))
    
    # [수정] 컨트롤 안내에 'L' 키 추가
    controls = ["Controls:", "  T : Trigger (Accel->Coast->Stop)", "  R : Reset", "  L : Length Analysis (Graph)", "  Q/E/W/S/Z/X : Camera"]
    for ln in controls:
        hud_surface.blit(font.render(ln, True, (20,20,80)), (x0, y))
        y += 18
    y += 10
    
    params = [
        f"Lift Status: {'LIFTING' if is_lifting else 'DONE'}",
        f"Motion Phase: {status_str}",
        f"Current Length: {L:.2f} m",
        f"Target Height: {target_height:.2f} m",
        f"Velocity: {crane_v:.2f} m/s",
        f"Swing Angle: {math.degrees(theta):.2f}°",
        f"Time: {time_elapsed:.2f} s",
    ]
    for p in params:
        c = color if "Phase" in p else (0,0,0)
        hud_surface.blit(font.render(p, True, c), (x0, y))
        y += 18
    y += 10

    if has_results:
        box_h = 90
        pygame.draw.rect(hud_surface, (230,255,230,255), (x0, y, HUD_W-2*m, box_h))
        pygame.draw.rect(hud_surface, (0,150,0,255), (x0, y, HUD_W-2*m, box_h), 2)
        yt = y + 5
        hud_surface.blit(font.render("Experiment Results:", True, (0,0,0)), (x0+5, yt))
        yt += 20
        res = [f"Max Swing: {max_angle_deg:.2f}°", f"Lifting E: {lifting_energy:.1f} J", 
               f"Control E: {abs(control_energy_net):.1f} J", f"Total E: {total_energy_required:.1f} J"]
        for r in res:
            hud_surface.blit(font.render(r, True, (0,0,0)), (x0+5, yt))
            yt += 16
        y += box_h + 10

    box_w, box_h = HUD_W - 2*m, 28
    length_y = HUD_H - m - box_h
    height_y = length_y - 10 - box_h

    def draw_input(label, val_str, local_y, is_active):
        color = (0,0,200) if is_active else (0,0,0)
        hud_surface.blit(font.render(label, True, (0,0,0)), (x0, local_y - 18))
        pygame.draw.rect(hud_surface, (255,255,255,255), (x0, local_y, box_w, box_h))
        pygame.draw.rect(hud_surface, color, (x0, local_y, box_w, box_h), 2)
        hud_surface.blit(font.render(val_str, True, (0,0,0)), (x0 + 8, local_y + 5))

    disp_H = input_height_text if input_height_text else f"{target_height:.2f}"
    draw_input("Target Height (m):", disp_H, height_y, active_field=="height")
    disp_L = input_length_text if input_length_text else f"{target_cable_length:.2f}"
    draw_input("Target Cable Length (m):", disp_L, length_y, active_field=="length")

    panel_x0 = WIDTH - HUD_W
    height_rect = pygame.Rect(panel_x0 + x0, height_y, box_w, box_h)
    length_rect = pygame.Rect(panel_x0 + x0, length_y, box_w, box_h)

def draw_hud():
    update_hud_surface()
    glEnable(GL_TEXTURE_2D)
    
    # 1. Main HUD
    glBindTexture(GL_TEXTURE_2D, hud_texture_id)
    hud_data = pygame.image.tostring(hud_surface, "RGBA", True)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, HUD_W, HUD_H, 0, GL_RGBA, GL_UNSIGNED_BYTE, hud_data)

    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity(); glOrtho(0, WIDTH, 0, HEIGHT, -1, 1)
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity(); glDisable(GL_DEPTH_TEST)
    
    glColor4f(1,1,1,1)
    glBegin(GL_QUADS)
    glTexCoord2f(0,0); glVertex2f(WIDTH - HUD_W, 0)
    glTexCoord2f(1,0); glVertex2f(WIDTH, 0)
    glTexCoord2f(1,1); glVertex2f(WIDTH, HEIGHT)
    glTexCoord2f(0,1); glVertex2f(WIDTH - HUD_W, HEIGHT)
    glEnd()

    # 2. Real-time Graph (Mini)
    if graph_surface:
        glBindTexture(GL_TEXTURE_2D, graph_texture_id)
        gw, gh = 320, 240
        gx, gy = 20, 20
        glColor4f(1,1,1,1)
        glBegin(GL_QUADS)
        glTexCoord2f(0,1); glVertex2f(gx, gy)
        glTexCoord2f(1,1); glVertex2f(gx+gw, gy)
        glTexCoord2f(1,0); glVertex2f(gx+gw, gy+gh)
        glTexCoord2f(0,0); glVertex2f(gx, gy+gh)
        glEnd()

    # [추가] 3. Analysis Graph (Overlay)
    if show_length_analysis_graph and length_analysis_data:
        glBindTexture(GL_TEXTURE_2D, length_analysis_texture_id)
        # 화면 중앙에 크게 표시
        margin = 100
        ow = WIDTH - 2*margin
        oh = HEIGHT - 2*margin
        ox, oy = margin, margin
        
        # 배경 (반투명 검정)
        glColor4f(0,0,0,0.7)
        glDisable(GL_TEXTURE_2D)
        glBegin(GL_QUADS)
        glVertex2f(0,0); glVertex2f(WIDTH,0)
        glVertex2f(WIDTH,HEIGHT); glVertex2f(0,HEIGHT)
        glEnd()
        glEnable(GL_TEXTURE_2D)
        
        # 그래프
        glColor4f(1,1,1,1)
        glBegin(GL_QUADS)
        glTexCoord2f(0,1); glVertex2f(ox, oy)
        glTexCoord2f(1,1); glVertex2f(ox+ow, oy)
        glTexCoord2f(1,0); glVertex2f(ox+ow, oy+oh)
        glTexCoord2f(0,0); glVertex2f(ox, oy+oh)
        glEnd()

    glEnable(GL_DEPTH_TEST)
    glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix(); glMatrixMode(GL_MODELVIEW)
    glBindTexture(GL_TEXTURE_2D, 0); glDisable(GL_TEXTURE_2D)

def reset_simulation():
    global crane_x, crane_v, theta, omega, time_elapsed, trial_running, trial_phase, trial_time
    global has_results, max_angle_deg, max_horizontal_disp, L, is_lifting, lifting_completed
    global lifting_energy, control_energy_net, total_energy_required, energy_history
    global show_length_analysis_graph

    crane_x, crane_v, theta, omega = 0.0, 0.0, 0.0, 0.0
    time_elapsed = 0.0
    trial_running, trial_phase, trial_time = False, "idle", 0.0
    L, is_lifting, lifting_completed = L_total, True, False
    has_results, max_angle_deg, max_horizontal_disp = False, 0.0, 0.0
    lifting_energy, control_energy_net, total_energy_required = 0.0, 0.0, 0.0
    energy_history = []
    show_length_analysis_graph = False

def init():
    pygame.init()
    pygame.display.set_mode((WIDTH, HEIGHT), DOUBLEBUF | OPENGL)
    pygame.display.set_caption("3D Port Crane - Energy Analysis")
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glClearColor(0.95, 0.95, 1.0, 1.0)
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); gluPerspective(45.0, WIDTH/HEIGHT, 0.1, 100.0)
    init_hud()

def main():
    global crane_v, cam_yaw, cam_pitch, cam_dist, external_accel, input_length_text, input_height_text, active_field
    global trial_running, trial_phase, trial_time, has_results, target_cable_length, target_height
    global show_length_analysis_graph

    init()
    clock = pygame.time.Clock()
    running = True
    graph_update_timer = 0

    print("="*70)
    print("3D Port Crane Energy Analysis System")
    print("Sequence: Accel(1s) -> Coast(3s) -> Decel(1s) -> Stop")
    print("Controls: T(Trigger), R(Reset), L(Analyze Graph), Camera(Q/E/W/S/Z/X)")
    print("="*70)

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE: running = False
                elif event.key == pygame.K_r: reset_simulation()
                
                # [추가] 'L' 키: 길이별 에너지 분석 실행 및 그래프 토글
                elif event.key == pygame.K_l:
                    if not length_analysis_data:
                        # 데이터가 없으면 분석 실행
                        analyze_length_vs_energy()
                        show_length_analysis_graph = True
                    else:
                        # 데이터가 있으면 토글
                        show_length_analysis_graph = not show_length_analysis_graph
                
                elif event.key == pygame.K_t:
                    if lifting_completed:
                        trial_running, trial_phase, trial_time, has_results = True, "accelerate", 0.0, True
                elif event.key == pygame.K_RETURN:
                    if active_field == "length" and input_length_text.strip():
                        try:
                            val = float(input_length_text)
                            if 0.1 <= val <= L_total:
                                target_cable_length = val
                                target_height = L_total - target_cable_length
                                reset_simulation()
                        except: pass
                        input_length_text = ""
                    elif active_field == "height" and input_height_text.strip():
                        try:
                            val = float(input_height_text)
                            if 0 <= val <= L_total:
                                target_height = val
                                target_cable_length = L_total - target_height
                                reset_simulation()
                        except: pass
                        input_height_text = ""
                elif event.key == pygame.K_BACKSPACE:
                    if active_field == "length": input_length_text = input_length_text[:-1]
                    elif active_field == "height": input_height_text = input_height_text[:-1]
                else:
                    if event.unicode in "0123456789.-":
                        if active_field == "length": input_length_text += event.unicode
                        elif active_field == "height": input_height_text += event.unicode
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = event.pos
                if length_rect and length_rect.collidepoint(mx, my): active_field = "length"
                elif height_rect and height_rect.collidepoint(mx, my): active_field = "height"

        keys = pygame.key.get_pressed()
        if keys[K_q]: cam_yaw -= 0.02
        if keys[K_e]: cam_yaw += 0.02
        if keys[K_w]: cam_pitch += 0.02
        if keys[K_s]: cam_pitch -= 0.02
        cam_pitch = max(-math.pi/2+0.1, min(math.pi/2-0.1, cam_pitch))
        if keys[K_z]: cam_dist = max(10.0, cam_dist-0.4)
        if keys[K_x]: cam_dist = min(70.0, cam_dist+0.4)

        if trial_running and lifting_completed:
            trial_time += dt
            if trial_phase == "accelerate":
                external_accel = INITIAL_ACCEL
                if trial_time >= ACCEL_DURATION:
                    trial_phase = "coast"
                    trial_time = 0.0
            elif trial_phase == "coast":
                external_accel = 0.0
                if trial_time >= COAST_DURATION:
                    trial_phase = "decelerate"
                    trial_time = 0.0
            elif trial_phase == "decelerate":
                external_accel = -INITIAL_ACCEL
                if crane_v <= 0 or trial_time >= DECEL_DURATION:
                    trial_phase = "hold"
                    external_accel = 0.0
                    crane_v = 0.0
            elif trial_phase == "hold":
                external_accel = 0.0
        else:
            external_accel = 0.0

        update_physics()
        
        graph_update_timer += 1
        if graph_update_timer > 10:
            update_graph_texture()
            graph_update_timer = 0

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_PROJECTION); glLoadIdentity(); gluPerspective(45.0, WIDTH/HEIGHT, 0.1, 100.0)
        set_camera()
        draw_scene()
        draw_hud()

        pygame.display.flip()
        clock.tick(int(1/dt))

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()