# import pygame
# import numpy as np
# import sys
# import matplotlib.pyplot as plt

# # ---------------------------------------------------------
# # 초기화 및 설정
# # ---------------------------------------------------------
# pygame.init()

# WIDTH, HEIGHT = 1400, 900
# screen = pygame.display.set_mode((WIDTH, HEIGHT))
# pygame.display.set_caption("Hybrid Crane Control: Trolley Motion + Active Cable Tension")
# clock = pygame.time.Clock()

# # 색상 정의
# WHITE = (255, 255, 255)
# BLACK = (0, 0, 0)
# RED = (200, 50, 50)
# BLUE = (50, 100, 200)
# GRAY = (180, 180, 180)
# GREEN = (0, 150, 0)
# DARK_GRAY = (60, 60, 60)
# ORANGE = (255, 140, 0)

# # 폰트 설정
# try:
#     font = pygame.font.SysFont("arial", 24)
#     tiny_font = pygame.font.SysFont("arial", 16)
# except:
#     font = pygame.font.Font(None, 24)
#     tiny_font = pygame.font.Font(None, 16)

# # ---------------------------------------------------------
# # 1. 물리 및 시스템 상수
# # ---------------------------------------------------------
# g = 9.81
# L_NOMINAL = 4.0   # 기본 케이블 길이 (m)
# m = 1000.0        # 질량 (kg)
# SCALE = 60        # 픽셀/미터 비율
# CABLE_SEP = 1.2   # 트롤리 상단 케이블 간격 (m)

# # 목표 설정
# start_x = 2.0
# target_distance = 18.0
# final_x = start_x + target_distance

# # 시뮬레이션 설정
# dt = 0.01
# max_simulation_time = 40.0

# # ---------------------------------------------------------
# # 2. 제어 게인 튜닝 (Hybrid Control)
# # ---------------------------------------------------------
# # A. 트롤리 모션 제어 (주행 담당)
# Kp_pos = 2.5
# Kd_pos = 2.0

# # B. 케이블 차동 제어 (Sway 억제 담당)
# Kp_cable = 5.0     # 각도 오차에 대한 보정 게인
# Kd_cable = 2.5     # 각속도 댐핑 게인
# Kf_cable = 0.95    # 피드포워드 게인 (관성력 상쇄 핵심)

# # 시스템 제한
# MAX_ACCEL = 0.8  # m/s²
# MAX_VEL = 2.5    # m/s
# MAX_CABLE_DIFF = 0.5 # 케이블 길이 차이 최대 한계 (m)

# # ---------------------------------------------------------
# # 3. 경로 생성 (S-Curve Profile)
# # ---------------------------------------------------------
# def generate_s_curve_profile(distance, max_v, max_a):
#     t_acc = max_v / max_a
#     d_acc = 0.5 * max_a * t_acc**2
    
#     if d_acc * 2 > distance:
#         d_acc = distance / 2
#         t_acc = np.sqrt(2 * d_acc / max_a)
#         max_v = max_a * t_acc
#         t_dec = t_acc
#         t_cruise = 0
#     else:
#         d_cruise = distance - 2 * d_acc
#         t_cruise = d_cruise / max_v
#         t_dec = t_acc
        
#     total_time = t_acc + t_cruise + t_dec
#     return t_acc, t_cruise, t_dec, max_v, max_a

# p_ta, p_tc, p_td, p_v, p_a = generate_s_curve_profile(target_distance, MAX_VEL, MAX_ACCEL)

# def get_reference(t):
#     if t < p_ta: # 가속
#         ref_a = p_a
#         ref_v = p_a * t
#         ref_x = start_x + 0.5 * p_a * t**2
#         phase = "ACCEL"
#     elif t < p_ta + p_tc: # 등속
#         dt_cruise = t - p_ta
#         dist_acc = 0.5 * p_a * p_ta**2
#         ref_a = 0
#         ref_v = p_v
#         ref_x = start_x + dist_acc + p_v * dt_cruise
#         phase = "CRUISE"
#     elif t < p_ta + p_tc + p_td: # 감속
#         dt_dec = t - (p_ta + p_tc)
#         dist_acc = 0.5 * p_a * p_ta**2
#         dist_cruise = p_v * p_tc
#         ref_a = -p_a
#         ref_v = p_v - p_a * dt_dec
#         ref_x = start_x + dist_acc + dist_cruise + (p_v * dt_dec - 0.5 * p_a * dt_dec**2)
#         phase = "DECEL"
#     else: # 정지
#         ref_a = 0
#         ref_v = 0
#         ref_x = final_x
#         phase = "STOP"
#     return ref_x, ref_v, ref_a, phase

# # ---------------------------------------------------------
# # 4. 상태 변수 초기화
# # ---------------------------------------------------------
# crane_x = start_x
# crane_v = 0.0
# theta = 0.0
# omega = 0.0
# L_left = L_NOMINAL
# L_right = L_NOMINAL
# cable_diff = 0.0

# time_elapsed = 0.0
# applied_accel = 0.0

# history = {'time':[], 'pos':[], 'vel':[], 'theta':[], 'accel':[], 'diff':[]}

# # ---------------------------------------------------------
# # 메인 루프
# # ---------------------------------------------------------
# running = True
# print("=== Hybrid Crane Control Simulation ===")
# print("시스템 초기화 완료. 시뮬레이션 시작.")

# while running:
#     # 이벤트 처리
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False
#         if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
#             running = False

#     if time_elapsed < max_simulation_time:
#         # [Step 1] Reference
#         ref_x, ref_v, ref_a, phase = get_reference(time_elapsed)
#         error_pos = ref_x - crane_x
#         error_vel = ref_v - crane_v
        
#         # [Step 2] Trolley Motion
#         trolley_accel_cmd = ref_a + (Kp_pos * error_pos) + (Kd_pos * error_vel)
#         trolley_accel_cmd = np.clip(trolley_accel_cmd, -MAX_ACCEL * 1.5, MAX_ACCEL * 1.5)
        
#         # [Step 3] Active Cable Control (Sway 0.05 rad 제한 핵심)
#         # 피드포워드: 가속도에 비례해 미리 당김
#         ff_term = (ref_a / g) * CABLE_SEP * Kf_cable
#         # 피드백: 오차 보정
#         fb_term = -(Kp_cable * theta) - (Kd_cable * omega)
        
#         target_diff = ff_term + fb_term
#         target_diff = np.clip(target_diff, -MAX_CABLE_DIFF, MAX_CABLE_DIFF)
        
#         # 케이블 길이 적용
#         cable_diff = target_diff
#         L_left = L_NOMINAL + cable_diff / 2
#         L_right = L_NOMINAL - cable_diff / 2
        
#         # [Step 4] Physics
#         applied_accel = trolley_accel_cmd
#         crane_v += applied_accel * dt
#         crane_x += crane_v * dt
        
#         # 케이블 비대칭에 의한 수평 복원력(g*sin(phi)) 추가
#         phi = np.arctan(cable_diff / CABLE_SEP)
#         L_avg = (L_left + L_right) / 2
        
#         alpha = (-g * np.sin(theta) - applied_accel * np.cos(theta) + g * np.sin(phi)) / L_avg
#         alpha -= 0.1 * omega # 마찰
        
#         omega += alpha * dt
#         theta += omega * dt
        
#         time_elapsed += dt
        
#         # 기록
#         history['time'].append(time_elapsed)
#         history['pos'].append(crane_x)
#         history['vel'].append(crane_v)
#         history['theta'].append(theta)
#         history['accel'].append(applied_accel)
#         history['diff'].append(cable_diff)

#     # --- 화면 그리기 ---
#     screen.fill(WHITE)
    
#     # 지면
#     pygame.draw.line(screen, BLACK, (0, HEIGHT-50), (WIDTH, HEIGHT-50), 2)
    
#     # 좌표 변환
#     center_offset = 100
#     draw_crane_x = int(crane_x * SCALE) + center_offset
#     draw_crane_y = 200
    
#     # 트롤리
#     pygame.draw.rect(screen, DARK_GRAY, (draw_crane_x - 60, draw_crane_y - 30, 120, 30))
#     pygame.draw.circle(screen, BLACK, (draw_crane_x - 40, draw_crane_y - 30), 10)
#     pygame.draw.circle(screen, BLACK, (draw_crane_x + 40, draw_crane_y - 30), 10)
#     pygame.draw.line(screen, GRAY, (0, draw_crane_y), (WIDTH, draw_crane_y), 3)
    
#     # 케이블 부착점
#     att_l_x = draw_crane_x - int(CABLE_SEP * SCALE / 2)
#     att_r_x = draw_crane_x + int(CABLE_SEP * SCALE / 2)
#     att_y = draw_crane_y
    
#     # 컨테이너 위치
#     cont_x = draw_crane_x + int(L_NOMINAL * SCALE * np.sin(theta))
#     cont_y = draw_crane_y + int(L_NOMINAL * SCALE * np.cos(theta))
    
#     # 케이블 그리기
#     c_color = RED if abs(cable_diff) > 0.01 else BLACK
#     pygame.draw.line(screen, c_color, (att_l_x, att_y), (cont_x - 20, cont_y - 20), 2)
#     pygame.draw.line(screen, c_color, (att_r_x, att_y), (cont_x + 20, cont_y - 20), 2)
    
#     # 컨테이너
#     is_safe = abs(theta) < 0.05
#     cont_color = GREEN if is_safe else ORANGE
#     if abs(theta) > 0.05: cont_color = RED
    
#     pygame.draw.rect(screen, cont_color, (cont_x - 30, cont_y - 20, 60, 40))
#     pygame.draw.rect(screen, BLACK, (cont_x - 30, cont_y - 20, 60, 40), 2)
    
#     # 목표 위치
#     target_draw_x = int(final_x * SCALE) + center_offset
#     pygame.draw.line(screen, BLUE, (target_draw_x, draw_crane_y), (target_draw_x, HEIGHT-50), 1)
    
#     # 정보 텍스트
#     infos = [
#         f"Hybrid Control: Trolley Motion + Active Cables",
#         f"Phase: {phase}",
#         f"Time: {time_elapsed:.2f} s",
#         f"Cable Diff: {cable_diff*1000:.1f} mm",
#         f"Sway: {theta:.4f} rad (Limit: 0.05)",
#         f"Accel: {applied_accel:.2f} m/s²"
#     ]
    
#     for i, txt in enumerate(infos):
#         color = BLACK
#         if "Sway" in txt and not is_safe: color = RED
#         if "Cable" in txt and abs(cable_diff) > 0.01: color = RED
#         surf = font.render(txt, True, color)
#         screen.blit(surf, (20, 20 + i * 25))

#     # 그래프 1: Sway
#     g_x, g_y, g_w, g_h = 50, 600, 400, 120
#     pygame.draw.rect(screen, (245, 245, 245), (g_x, g_y, g_w, g_h))
#     pygame.draw.rect(screen, BLACK, (g_x, g_y, g_w, g_h), 1)
#     screen.blit(tiny_font.render("Sway Angle (rad) [Limit 0.05]", True, BLACK), (g_x+5, g_y+5))
    
#     limit_y_top = g_y + g_h/2 - (0.05 * 500)
#     limit_y_bot = g_y + g_h/2 - (-0.05 * 500)
#     pygame.draw.line(screen, RED, (g_x, limit_y_top), (g_x+g_w, limit_y_top), 1)
#     pygame.draw.line(screen, RED, (g_x, limit_y_bot), (g_x+g_w, limit_y_bot), 1)

#     if len(history['theta']) > 1:
#         pts = []
#         for i in range(len(history['theta'])):
#             px = g_x + (history['time'][i] / max_simulation_time) * g_w
#             py = g_y + g_h/2 - (history['theta'][i] * 500)
#             pts.append((px, py))
#             if px > g_x + g_w: break
#         if len(pts) > 1:
#             pygame.draw.lines(screen, BLUE, False, pts, 2)

#     # 그래프 2: Cable Diff
#     g_y2 = g_y + 140
#     pygame.draw.rect(screen, (245, 245, 245), (g_x, g_y2, g_w, g_h))
#     pygame.draw.rect(screen, BLACK, (g_x, g_y2, g_w, g_h), 1)
#     screen.blit(tiny_font.render("Cable Diff (m)", True, BLACK), (g_x+5, g_y2+5))
    
#     if len(history['diff']) > 1:
#         pts = []
#         for i in range(len(history['diff'])):
#             px = g_x + (history['time'][i] / max_simulation_time) * g_w
#             py = g_y2 + g_h/2 - (history['diff'][i] * 100)
#             pts.append((px, py))
#             if px > g_x + g_w: break
#         if len(pts) > 1:
#             pygame.draw.lines(screen, RED, False, pts, 2)

#     pygame.display.flip()
#     clock.tick(60)

# pygame.quit()

# # 결과 그래프 저장
# plt.figure(figsize=(10, 10))

# plt.subplot(3, 1, 1)
# plt.plot(history['time'], history['accel'], 'g', label='Trolley Accel')
# plt.ylabel('Accel (m/s^2)')
# plt.title('Trolley Acceleration')
# plt.grid(True)

# plt.subplot(3, 1, 2)
# plt.plot(history['time'], history['diff'], 'r', label='Cable Diff')
# plt.ylabel('Cable Length Diff (m)')
# plt.title('Active Cable Control')
# plt.grid(True)

# plt.subplot(3, 1, 3)
# plt.plot(history['time'], history['theta'], 'b')
# plt.axhline(y=0.05, color='r', linestyle='--', label='Limit (0.05)')
# plt.axhline(y=-0.05, color='r', linestyle='--')
# plt.ylabel('Sway (rad)')
# plt.title('Sway Angle Result')
# plt.legend()
# plt.grid(True)

# plt.tight_layout()
# plt.savefig('hybrid_crane_control.png')
# print("Simulation Finished. Graph saved.")











































































































































































# import pygame
# import numpy as np
# import sys
# import matplotlib.pyplot as plt

# # ---------------------------------------------------------
# # 초기화 및 설정
# # ---------------------------------------------------------
# pygame.init()

# WIDTH, HEIGHT = 1400, 900
# screen = pygame.display.set_mode((WIDTH, HEIGHT))
# pygame.display.set_caption("Hybrid Crane Control with Tension Analysis")
# clock = pygame.time.Clock()

# # 색상 정의
# WHITE = (255, 255, 255)
# BLACK = (0, 0, 0)
# RED = (200, 50, 50)
# BLUE = (50, 100, 200)
# GRAY = (180, 180, 180)
# GREEN = (0, 150, 0)
# DARK_GRAY = (60, 60, 60)
# ORANGE = (255, 140, 0)
# PURPLE = (128, 0, 128)

# # 폰트 설정
# try:
#     font = pygame.font.SysFont("arial", 24)
#     tiny_font = pygame.font.SysFont("arial", 16)
#     bold_font = pygame.font.SysFont("arial", 20, bold=True)
# except:
#     font = pygame.font.Font(None, 24)
#     tiny_font = pygame.font.Font(None, 16)
#     bold_font = pygame.font.Font(None, 20)

# # ---------------------------------------------------------
# # 1. 물리 및 시스템 상수
# # ---------------------------------------------------------
# g = 9.81
# L_NOMINAL = 4.0   # 기본 케이블 길이 (m)
# m = 1000.0        # 질량 (kg)
# SCALE = 60        # 픽셀/미터 비율
# CABLE_SEP = 1.2   # 트롤리 상단 케이블 간격 (m)

# # 목표 설정
# start_x = 2.0
# target_distance = 18.0
# final_x = start_x + target_distance

# # 시뮬레이션 설정
# dt = 0.01
# max_simulation_time = 40.0

# # ---------------------------------------------------------
# # 2. 제어 게인 튜닝
# # ---------------------------------------------------------
# Kp_pos = 2.5
# Kd_pos = 2.0

# Kp_cable = 5.0
# Kd_cable = 2.5
# Kf_cable = 0.95

# MAX_ACCEL = 0.8
# MAX_VEL = 2.5
# MAX_CABLE_DIFF = 0.5

# # ---------------------------------------------------------
# # 3. 경로 생성 (S-Curve)
# # ---------------------------------------------------------
# def generate_s_curve_profile(distance, max_v, max_a):
#     t_acc = max_v / max_a
#     d_acc = 0.5 * max_a * t_acc**2
    
#     if d_acc * 2 > distance:
#         d_acc = distance / 2
#         t_acc = np.sqrt(2 * d_acc / max_a)
#         max_v = max_a * t_acc
#         t_dec = t_acc
#         t_cruise = 0
#     else:
#         d_cruise = distance - 2 * d_acc
#         t_cruise = d_cruise / max_v
#         t_dec = t_acc
        
#     total_time = t_acc + t_cruise + t_dec
#     return t_acc, t_cruise, t_dec, max_v, max_a

# p_ta, p_tc, p_td, p_v, p_a = generate_s_curve_profile(target_distance, MAX_VEL, MAX_ACCEL)

# def get_reference(t):
#     if t < p_ta:
#         ref_a = p_a
#         ref_v = p_a * t
#         ref_x = start_x + 0.5 * p_a * t**2
#         phase = "ACCEL"
#     elif t < p_ta + p_tc:
#         dt_cruise = t - p_ta
#         dist_acc = 0.5 * p_a * p_ta**2
#         ref_a = 0
#         ref_v = p_v
#         ref_x = start_x + dist_acc + p_v * dt_cruise
#         phase = "CRUISE"
#     elif t < p_ta + p_tc + p_td:
#         dt_dec = t - (p_ta + p_tc)
#         dist_acc = 0.5 * p_a * p_ta**2
#         dist_cruise = p_v * p_tc
#         ref_a = -p_a
#         ref_v = p_v - p_a * dt_dec
#         ref_x = start_x + dist_acc + dist_cruise + (p_v * dt_dec - 0.5 * p_a * dt_dec**2)
#         phase = "DECEL"
#     else:
#         ref_a = 0
#         ref_v = 0
#         ref_x = final_x
#         phase = "STOP"
#     return ref_x, ref_v, ref_a, phase

# # ---------------------------------------------------------
# # 4. 상태 변수 초기화
# # ---------------------------------------------------------
# crane_x = start_x
# crane_v = 0.0
# theta = 0.0
# omega = 0.0
# L_left = L_NOMINAL
# L_right = L_NOMINAL
# cable_diff = 0.0

# T_left = 0.0
# T_right = 0.0

# time_elapsed = 0.0
# applied_accel = 0.0

# history = {'time':[], 'pos':[], 'vel':[], 'theta':[], 'accel':[], 'diff':[], 'Tl':[], 'Tr':[]}

# # ---------------------------------------------------------
# # 메인 루프
# # ---------------------------------------------------------
# running = True
# print("=== Hybrid Crane Control Simulation ===")
# print("장력(Tension) 모니터링 기능 활성화됨.")

# while running:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False
#         if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
#             running = False

#     if time_elapsed < max_simulation_time:
#         # [Step 1] Reference
#         ref_x, ref_v, ref_a, phase = get_reference(time_elapsed)
#         error_pos = ref_x - crane_x
#         error_vel = ref_v - crane_v
        
#         # [Step 2] Trolley Motion
#         trolley_accel_cmd = ref_a + (Kp_pos * error_pos) + (Kd_pos * error_vel)
#         trolley_accel_cmd = np.clip(trolley_accel_cmd, -MAX_ACCEL * 1.5, MAX_ACCEL * 1.5)
        
#         # [Step 3] Active Cable Control
#         ff_term = (ref_a / g) * CABLE_SEP * Kf_cable
#         fb_term = -(Kp_cable * theta) - (Kd_cable * omega)
        
#         target_diff = ff_term + fb_term
#         target_diff = np.clip(target_diff, -MAX_CABLE_DIFF, MAX_CABLE_DIFF)
        
#         cable_diff = target_diff
#         L_left = L_NOMINAL + cable_diff / 2
#         L_right = L_NOMINAL - cable_diff / 2
        
#         # [Step 4] Tension Calculation (장력 계산)
#         # 기본 하중: F = m * sqrt(g^2 + a^2) (관성력 포함한 겉보기 무게)
#         total_load = m * np.sqrt(g**2 + applied_accel**2)
        
#         # 하중 분배: 케이블이 짧은 쪽이 더 많은 하중을 부담함
#         # 케이블 비대칭 비율 (-1.0 ~ 1.0)
#         load_bias = cable_diff / CABLE_SEP 
        
#         # Bias가 양수(Left Long, Right Short) -> Right Tension 증가
#         # 증폭 계수 2.0 (민감도)
#         T_left = (total_load / 2) * (1 - load_bias * 2.5)
#         T_right = (total_load / 2) * (1 + load_bias * 2.5)
        
#         # 최소 장력 0 (케이블 처짐)
#         T_left = max(100.0, T_left)
#         T_right = max(100.0, T_right)

#         # [Step 5] Physics
#         applied_accel = trolley_accel_cmd
#         crane_v += applied_accel * dt
#         crane_x += crane_v * dt
        
#         phi = np.arctan(cable_diff / CABLE_SEP)
#         L_avg = (L_left + L_right) / 2
        
#         alpha = (-g * np.sin(theta) - applied_accel * np.cos(theta) + g * np.sin(phi)) / L_avg
#         alpha -= 0.1 * omega
        
#         omega += alpha * dt
#         theta += omega * dt
        
#         time_elapsed += dt
        
#         # 기록
#         history['time'].append(time_elapsed)
#         history['pos'].append(crane_x)
#         history['vel'].append(crane_v)
#         history['theta'].append(theta)
#         history['accel'].append(applied_accel)
#         history['diff'].append(cable_diff)
#         history['Tl'].append(T_left)
#         history['Tr'].append(T_right)

#     # --- 화면 그리기 ---
#     screen.fill(WHITE)
#     pygame.draw.line(screen, BLACK, (0, HEIGHT-50), (WIDTH, HEIGHT-50), 2)
    
#     center_offset = 100
#     draw_crane_x = int(crane_x * SCALE) + center_offset
#     draw_crane_y = 200
    
#     # 트롤리
#     pygame.draw.rect(screen, DARK_GRAY, (draw_crane_x - 60, draw_crane_y - 30, 120, 30))
#     pygame.draw.circle(screen, BLACK, (draw_crane_x - 40, draw_crane_y - 30), 10)
#     pygame.draw.circle(screen, BLACK, (draw_crane_x + 40, draw_crane_y - 30), 10)
#     pygame.draw.line(screen, GRAY, (0, draw_crane_y), (WIDTH, draw_crane_y), 3)
    
#     att_l_x = draw_crane_x - int(CABLE_SEP * SCALE / 2)
#     att_r_x = draw_crane_x + int(CABLE_SEP * SCALE / 2)
#     att_y = draw_crane_y
    
#     cont_x = draw_crane_x + int(L_NOMINAL * SCALE * np.sin(theta))
#     cont_y = draw_crane_y + int(L_NOMINAL * SCALE * np.cos(theta))
    
#     # 케이블
#     c_color = RED if abs(cable_diff) > 0.01 else BLACK
#     pygame.draw.line(screen, c_color, (att_l_x, att_y), (cont_x - 20, cont_y - 20), 2)
#     pygame.draw.line(screen, c_color, (att_r_x, att_y), (cont_x + 20, cont_y - 20), 2)
    
#     # --- 장력 텍스트 표시 (케이블 옆에) ---
#     tl_text = bold_font.render(f"{int(T_left)} N", True, BLUE)
#     tr_text = bold_font.render(f"{int(T_right)} N", True, PURPLE)
    
#     screen.blit(tl_text, (att_l_x - 60, att_y + 40))
#     screen.blit(tr_text, (att_r_x + 10, att_y + 40))
    
#     # 컨테이너
#     is_safe = abs(theta) < 0.05
#     cont_color = GREEN if is_safe else ORANGE
#     if abs(theta) > 0.05: cont_color = RED
    
#     pygame.draw.rect(screen, cont_color, (cont_x - 30, cont_y - 20, 60, 40))
#     pygame.draw.rect(screen, BLACK, (cont_x - 30, cont_y - 20, 60, 40), 2)
    
#     # 목표 위치
#     target_draw_x = int(final_x * SCALE) + center_offset
#     pygame.draw.line(screen, BLUE, (target_draw_x, draw_crane_y), (target_draw_x, HEIGHT-50), 1)
    
#     # 정보 텍스트
#     infos = [
#         f"Hybrid Control with Tension Monitor",
#         f"Phase: {phase}",
#         f"Time: {time_elapsed:.2f} s",
#         f"Cable Diff: {cable_diff*1000:.1f} mm",
#         f"Sway: {theta:.4f} rad (Limit: 0.05)",
#         f"Accel: {applied_accel:.2f} m/s²",
#         f"Total Load: {int(T_left + T_right)} N"
#     ]
    
#     for i, txt in enumerate(infos):
#         color = BLACK
#         if "Sway" in txt and not is_safe: color = RED
#         surf = font.render(txt, True, color)
#         screen.blit(surf, (20, 20 + i * 25))

#     # 그래프 1: Sway
#     g_x, g_y, g_w, g_h = 50, 600, 400, 120
#     pygame.draw.rect(screen, (245, 245, 245), (g_x, g_y, g_w, g_h))
#     pygame.draw.rect(screen, BLACK, (g_x, g_y, g_w, g_h), 1)
#     screen.blit(tiny_font.render("Sway Angle (rad)", True, BLACK), (g_x+5, g_y+5))
    
#     limit_y_top = g_y + g_h/2 - (0.05 * 500)
#     limit_y_bot = g_y + g_h/2 - (-0.05 * 500)
#     pygame.draw.line(screen, RED, (g_x, limit_y_top), (g_x+g_w, limit_y_top), 1)
#     pygame.draw.line(screen, RED, (g_x, limit_y_bot), (g_x+g_w, limit_y_bot), 1)

#     if len(history['theta']) > 1:
#         pts = []
#         for i in range(len(history['theta'])):
#             px = g_x + (history['time'][i] / max_simulation_time) * g_w
#             py = g_y + g_h/2 - (history['theta'][i] * 500)
#             pts.append((px, py))
#             if px > g_x + g_w: break
#         if len(pts) > 1:
#             pygame.draw.lines(screen, BLUE, False, pts, 2)

#     # 그래프 2: Cable Diff
#     g_y2 = g_y + 140
#     pygame.draw.rect(screen, (245, 245, 245), (g_x, g_y2, g_w, g_h))
#     pygame.draw.rect(screen, BLACK, (g_x, g_y2, g_w, g_h), 1)
#     screen.blit(tiny_font.render("Cable Diff (m)", True, BLACK), (g_x+5, g_y2+5))
    
#     if len(history['diff']) > 1:
#         pts = []
#         for i in range(len(history['diff'])):
#             px = g_x + (history['time'][i] / max_simulation_time) * g_w
#             py = g_y2 + g_h/2 - (history['diff'][i] * 100)
#             pts.append((px, py))
#             if px > g_x + g_w: break
#         if len(pts) > 1:
#             pygame.draw.lines(screen, RED, False, pts, 2)

#     pygame.display.flip()
#     clock.tick(60)

# pygame.quit()

# # 결과 그래프 저장 (2x2 레이아웃)
# plt.figure(figsize=(12, 10))

# plt.subplot(2, 2, 1)
# plt.plot(history['time'], history['accel'], 'g', label='Trolley Accel')
# plt.ylabel('Accel (m/s^2)')
# plt.title('1. Trolley Acceleration')
# plt.grid(True)

# plt.subplot(2, 2, 2)
# plt.plot(history['time'], history['diff'], 'r', label='Cable Diff')
# plt.ylabel('Cable Length Diff (m)')
# plt.title('2. Active Cable Control')
# plt.grid(True)

# plt.subplot(2, 2, 3)
# plt.plot(history['time'], history['theta'], 'b')
# plt.axhline(y=0.05, color='r', linestyle='--', label='Limit (0.05)')
# plt.axhline(y=-0.05, color='r', linestyle='--')
# plt.ylabel('Sway (rad)')
# plt.title('3. Sway Angle Result')
# plt.legend()
# plt.grid(True)

# # 4번째 그래프: 장력 변화
# plt.subplot(2, 2, 4)
# plt.plot(history['time'], history['Tl'], color='blue', label='Left Tension', alpha=0.7)
# plt.plot(history['time'], history['Tr'], color='purple', label='Right Tension', alpha=0.7)
# plt.ylabel('Tension (N)')
# plt.title('4. Cable Tensions (Load Distribution)')
# plt.legend()
# plt.grid(True)

# plt.tight_layout()
# plt.savefig('hybrid_crane_tension_analysis.png')
# print("Simulation Finished. Graph saved.")
















































































































































# import pygame
# import numpy as np
# import sys
# import matplotlib.pyplot as plt

# # ---------------------------------------------------------
# # 초기화 및 설정
# # ---------------------------------------------------------
# pygame.init()

# WIDTH, HEIGHT = 1400, 900
# screen = pygame.display.set_mode((WIDTH, HEIGHT))
# pygame.display.set_caption("High Acceleration Crane Control (1.5 m/s^2)")
# clock = pygame.time.Clock()

# # 색상 정의
# WHITE = (255, 255, 255)
# BLACK = (0, 0, 0)
# RED = (200, 50, 50)
# BLUE = (50, 100, 200)
# GRAY = (180, 180, 180)
# GREEN = (0, 150, 0)
# DARK_GRAY = (60, 60, 60)
# ORANGE = (255, 140, 0)
# PURPLE = (128, 0, 128)

# # 폰트 설정
# try:
#     font = pygame.font.SysFont("arial", 24)
#     tiny_font = pygame.font.SysFont("arial", 16)
#     bold_font = pygame.font.SysFont("arial", 20, bold=True)
# except:
#     font = pygame.font.Font(None, 24)
#     tiny_font = pygame.font.Font(None, 16)
#     bold_font = pygame.font.Font(None, 20)

# # ---------------------------------------------------------
# # 1. 물리 및 시스템 상수
# # ---------------------------------------------------------
# g = 9.81
# L_NOMINAL = 4.0   # 기본 케이블 길이 (m)
# m = 1000.0        # 질량 (kg)
# SCALE = 60        # 픽셀/미터 비율
# CABLE_SEP = 1.2   # 트롤리 상단 케이블 간격 (m)

# # 목표 설정
# start_x = 2.0
# target_distance = 18.0
# final_x = start_x + target_distance

# # 시뮬레이션 설정
# dt = 0.01
# max_simulation_time = 40.0

# # ---------------------------------------------------------
# # 2. 제어 게인 튜닝
# # ---------------------------------------------------------
# Kp_pos = 2.5
# Kd_pos = 2.0

# Kp_cable = 5.0
# Kd_cable = 2.5
# Kf_cable = 0.95

# # --- [수정된 부분] 가속도 상향 ---
# MAX_ACCEL = 1.5  # m/s² (0.8 -> 1.5로 변경)
# MAX_VEL = 3.0    # 가속도가 높으니 최대 속도도 조금 높임 (2.5 -> 3.0)
# MAX_CABLE_DIFF = 0.6 # 가속도가 커서 케이블 조절 범위도 약간 늘림

# # ---------------------------------------------------------
# # 3. 경로 생성 (S-Curve)
# # ---------------------------------------------------------
# def generate_s_curve_profile(distance, max_v, max_a):
#     t_acc = max_v / max_a
#     d_acc = 0.5 * max_a * t_acc**2
    
#     if d_acc * 2 > distance:
#         d_acc = distance / 2
#         t_acc = np.sqrt(2 * d_acc / max_a)
#         max_v = max_a * t_acc
#         t_dec = t_acc
#         t_cruise = 0
#     else:
#         d_cruise = distance - 2 * d_acc
#         t_cruise = d_cruise / max_v
#         t_dec = t_acc
        
#     total_time = t_acc + t_cruise + t_dec
#     return t_acc, t_cruise, t_dec, max_v, max_a

# p_ta, p_tc, p_td, p_v, p_a = generate_s_curve_profile(target_distance, MAX_VEL, MAX_ACCEL)

# def get_reference(t):
#     if t < p_ta:
#         ref_a = p_a
#         ref_v = p_a * t
#         ref_x = start_x + 0.5 * p_a * t**2
#         phase = "ACCEL"
#     elif t < p_ta + p_tc:
#         dt_cruise = t - p_ta
#         dist_acc = 0.5 * p_a * p_ta**2
#         ref_a = 0
#         ref_v = p_v
#         ref_x = start_x + dist_acc + p_v * dt_cruise
#         phase = "CRUISE"
#     elif t < p_ta + p_tc + p_td:
#         dt_dec = t - (p_ta + p_tc)
#         dist_acc = 0.5 * p_a * p_ta**2
#         dist_cruise = p_v * p_tc
#         ref_a = -p_a
#         ref_v = p_v - p_a * dt_dec
#         ref_x = start_x + dist_acc + dist_cruise + (p_v * dt_dec - 0.5 * p_a * dt_dec**2)
#         phase = "DECEL"
#     else:
#         ref_a = 0
#         ref_v = 0
#         ref_x = final_x
#         phase = "STOP"
#     return ref_x, ref_v, ref_a, phase

# # ---------------------------------------------------------
# # 4. 상태 변수 초기화
# # ---------------------------------------------------------
# crane_x = start_x
# crane_v = 0.0
# theta = 0.0
# omega = 0.0
# L_left = L_NOMINAL
# L_right = L_NOMINAL
# cable_diff = 0.0

# T_left = 0.0
# T_right = 0.0

# time_elapsed = 0.0
# applied_accel = 0.0

# history = {'time':[], 'pos':[], 'vel':[], 'theta':[], 'accel':[], 'diff':[], 'Tl':[], 'Tr':[]}

# # ---------------------------------------------------------
# # 메인 루프
# # ---------------------------------------------------------
# running = True
# print(f"=== Hybrid Crane Control Simulation (Accel: {MAX_ACCEL} m/s^2) ===")

# while running:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False
#         if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
#             running = False

#     if time_elapsed < max_simulation_time:
#         # [Step 1] Reference
#         ref_x, ref_v, ref_a, phase = get_reference(time_elapsed)
#         error_pos = ref_x - crane_x
#         error_vel = ref_v - crane_v
        
#         # [Step 2] Trolley Motion
#         trolley_accel_cmd = ref_a + (Kp_pos * error_pos) + (Kd_pos * error_vel)
#         trolley_accel_cmd = np.clip(trolley_accel_cmd, -MAX_ACCEL * 1.5, MAX_ACCEL * 1.5)
        
#         # [Step 3] Active Cable Control
#         ff_term = (ref_a / g) * CABLE_SEP * Kf_cable
#         fb_term = -(Kp_cable * theta) - (Kd_cable * omega)
        
#         target_diff = ff_term + fb_term
#         target_diff = np.clip(target_diff, -MAX_CABLE_DIFF, MAX_CABLE_DIFF)
        
#         cable_diff = target_diff
#         L_left = L_NOMINAL + cable_diff / 2
#         L_right = L_NOMINAL - cable_diff / 2
        
#         # [Step 4] Tension Calculation
#         total_load = m * np.sqrt(g**2 + applied_accel**2)
#         load_bias = cable_diff / CABLE_SEP 
        
#         T_left = (total_load / 2) * (1 - load_bias * 2.5)
#         T_right = (total_load / 2) * (1 + load_bias * 2.5)
        
#         T_left = max(50.0, T_left)
#         T_right = max(50.0, T_right)

#         # [Step 5] Physics
#         applied_accel = trolley_accel_cmd
#         crane_v += applied_accel * dt
#         crane_x += crane_v * dt
        
#         phi = np.arctan(cable_diff / CABLE_SEP)
#         L_avg = (L_left + L_right) / 2
        
#         alpha = (-g * np.sin(theta) - applied_accel * np.cos(theta) + g * np.sin(phi)) / L_avg
#         alpha -= 0.1 * omega
        
#         omega += alpha * dt
#         theta += omega * dt
        
#         time_elapsed += dt
        
#         # 기록
#         history['time'].append(time_elapsed)
#         history['pos'].append(crane_x)
#         history['vel'].append(crane_v)
#         history['theta'].append(theta)
#         history['accel'].append(applied_accel)
#         history['diff'].append(cable_diff)
#         history['Tl'].append(T_left)
#         history['Tr'].append(T_right)

#     # --- 화면 그리기 ---
#     screen.fill(WHITE)
#     pygame.draw.line(screen, BLACK, (0, HEIGHT-50), (WIDTH, HEIGHT-50), 2)
    
#     center_offset = 100
#     draw_crane_x = int(crane_x * SCALE) + center_offset
#     draw_crane_y = 200
    
#     # 트롤리
#     pygame.draw.rect(screen, DARK_GRAY, (draw_crane_x - 60, draw_crane_y - 30, 120, 30))
#     pygame.draw.circle(screen, BLACK, (draw_crane_x - 40, draw_crane_y - 30), 10)
#     pygame.draw.circle(screen, BLACK, (draw_crane_x + 40, draw_crane_y - 30), 10)
#     pygame.draw.line(screen, GRAY, (0, draw_crane_y), (WIDTH, draw_crane_y), 3)
    
#     att_l_x = draw_crane_x - int(CABLE_SEP * SCALE / 2)
#     att_r_x = draw_crane_x + int(CABLE_SEP * SCALE / 2)
#     att_y = draw_crane_y
    
#     cont_x = draw_crane_x + int(L_NOMINAL * SCALE * np.sin(theta))
#     cont_y = draw_crane_y + int(L_NOMINAL * SCALE * np.cos(theta))
    
#     # 케이블
#     c_color = RED if abs(cable_diff) > 0.01 else BLACK
#     pygame.draw.line(screen, c_color, (att_l_x, att_y), (cont_x - 20, cont_y - 20), 2)
#     pygame.draw.line(screen, c_color, (att_r_x, att_y), (cont_x + 20, cont_y - 20), 2)
    
#     # 장력 텍스트
#     tl_text = bold_font.render(f"{int(T_left)} N", True, BLUE)
#     tr_text = bold_font.render(f"{int(T_right)} N", True, PURPLE)
    
#     screen.blit(tl_text, (att_l_x - 60, att_y + 40))
#     screen.blit(tr_text, (att_r_x + 10, att_y + 40))
    
#     # 컨테이너
#     is_safe = abs(theta) < 0.05
#     cont_color = GREEN if is_safe else ORANGE
#     if abs(theta) > 0.05: cont_color = RED
    
#     pygame.draw.rect(screen, cont_color, (cont_x - 30, cont_y - 20, 60, 40))
#     pygame.draw.rect(screen, BLACK, (cont_x - 30, cont_y - 20, 60, 40), 2)
    
#     # 목표 위치
#     target_draw_x = int(final_x * SCALE) + center_offset
#     pygame.draw.line(screen, BLUE, (target_draw_x, draw_crane_y), (target_draw_x, HEIGHT-50), 1)
    
#     # 정보 텍스트
#     infos = [
#         f"Hybrid Control (Accel: {MAX_ACCEL} m/s^2)",
#         f"Phase: {phase}",
#         f"Time: {time_elapsed:.2f} s",
#         f"Cable Diff: {cable_diff*1000:.1f} mm",
#         f"Sway: {theta:.4f} rad (Limit: 0.05)",
#         f"Accel: {applied_accel:.2f} m/s²",
#         f"Load Bias: {int(T_right - T_left)} N"
#     ]
    
#     for i, txt in enumerate(infos):
#         color = BLACK
#         if "Sway" in txt and not is_safe: color = RED
#         surf = font.render(txt, True, color)
#         screen.blit(surf, (20, 20 + i * 25))

#     # 그래프 1: Sway
#     g_x, g_y, g_w, g_h = 50, 600, 400, 120
#     pygame.draw.rect(screen, (245, 245, 245), (g_x, g_y, g_w, g_h))
#     pygame.draw.rect(screen, BLACK, (g_x, g_y, g_w, g_h), 1)
#     screen.blit(tiny_font.render("Sway Angle (rad)", True, BLACK), (g_x+5, g_y+5))
    
#     limit_y_top = g_y + g_h/2 - (0.05 * 500)
#     limit_y_bot = g_y + g_h/2 - (-0.05 * 500)
#     pygame.draw.line(screen, RED, (g_x, limit_y_top), (g_x+g_w, limit_y_top), 1)
#     pygame.draw.line(screen, RED, (g_x, limit_y_bot), (g_x+g_w, limit_y_bot), 1)

#     if len(history['theta']) > 1:
#         pts = []
#         for i in range(len(history['theta'])):
#             px = g_x + (history['time'][i] / max_simulation_time) * g_w
#             py = g_y + g_h/2 - (history['theta'][i] * 500)
#             pts.append((px, py))
#             if px > g_x + g_w: break
#         if len(pts) > 1:
#             pygame.draw.lines(screen, BLUE, False, pts, 2)

#     # 그래프 2: Cable Diff
#     g_y2 = g_y + 140
#     pygame.draw.rect(screen, (245, 245, 245), (g_x, g_y2, g_w, g_h))
#     pygame.draw.rect(screen, BLACK, (g_x, g_y2, g_w, g_h), 1)
#     screen.blit(tiny_font.render("Cable Diff (m)", True, BLACK), (g_x+5, g_y2+5))
    
#     if len(history['diff']) > 1:
#         pts = []
#         for i in range(len(history['diff'])):
#             px = g_x + (history['time'][i] / max_simulation_time) * g_w
#             py = g_y2 + g_h/2 - (history['diff'][i] * 100)
#             pts.append((px, py))
#             if px > g_x + g_w: break
#         if len(pts) > 1:
#             pygame.draw.lines(screen, RED, False, pts, 2)

#     pygame.display.flip()
#     clock.tick(60)

# pygame.quit()

# # 결과 그래프 저장
# plt.figure(figsize=(12, 10))

# plt.subplot(2, 2, 1)
# plt.plot(history['time'], history['accel'], 'g', label='Trolley Accel')
# plt.ylabel('Accel (m/s^2)')
# plt.title(f'1. Trolley Acceleration (Max: {MAX_ACCEL})')
# plt.grid(True)

# plt.subplot(2, 2, 2)
# plt.plot(history['time'], history['diff'], 'r', label='Cable Diff')
# plt.ylabel('Cable Length Diff (m)')
# plt.title('2. Active Cable Control')
# plt.grid(True)

# plt.subplot(2, 2, 3)
# plt.plot(history['time'], history['theta'], 'b')
# plt.axhline(y=0.05, color='r', linestyle='--', label='Limit (0.05)')
# plt.axhline(y=-0.05, color='r', linestyle='--')
# plt.ylabel('Sway (rad)')
# plt.title('3. Sway Angle Result')
# plt.legend()
# plt.grid(True)

# plt.subplot(2, 2, 4)
# plt.plot(history['time'], history['Tl'], color='blue', label='Left Tension', alpha=0.7)
# plt.plot(history['time'], history['Tr'], color='purple', label='Right Tension', alpha=0.7)
# plt.ylabel('Tension (N)')
# plt.title('4. Cable Tensions')
# plt.legend()
# plt.grid(True)

# plt.tight_layout()
# plt.savefig('high_accel_crane_control.png')
# print("Simulation Finished. Graph saved.")
























































































































































# import pygame
# import numpy as np
# import sys
# import matplotlib.pyplot as plt

# # ---------------------------------------------------------
# # 초기화 및 설정
# # ---------------------------------------------------------
# pygame.init()

# WIDTH, HEIGHT = 1400, 900
# screen = pygame.display.set_mode((WIDTH, HEIGHT))
# pygame.display.set_caption("Crane Control with Elastic Cables (Hooke's Law)")
# clock = pygame.time.Clock()

# # 색상 정의
# WHITE = (255, 255, 255)
# BLACK = (0, 0, 0)
# RED = (200, 50, 50)
# BLUE = (50, 100, 200)
# GRAY = (180, 180, 180)
# GREEN = (0, 150, 0)
# DARK_GRAY = (60, 60, 60)
# ORANGE = (255, 140, 0)
# PURPLE = (128, 0, 128)

# # 폰트 설정
# try:
#     font = pygame.font.SysFont("arial", 24)
#     tiny_font = pygame.font.SysFont("arial", 16)
#     bold_font = pygame.font.SysFont("arial", 20, bold=True)
# except:
#     font = pygame.font.Font(None, 24)
#     tiny_font = pygame.font.Font(None, 16)
#     bold_font = pygame.font.Font(None, 20)

# # ---------------------------------------------------------
# # 1. 물리 및 시스템 상수
# # ---------------------------------------------------------
# g = 9.81
# L_NOMINAL = 4.0   # 기본 케이블 길이 (m)
# m = 1000.0        # 질량 (kg)
# SCALE = 60        # 픽셀/미터 비율
# CABLE_SEP = 1.2   # 트롤리 상단 케이블 간격 (m)

# # [추가됨] 케이블 탄성 특성 (Elastic Properties)
# # 실제 강철(200 GPa)은 너무 딱딱해서 시뮬레이션에서 티가 안 나므로,
# # 효과 확인을 위해 약간 부드러운 소재(약 50 GPa)로 가정하거나 단면적을 조정함.
# E_MODULUS = 50e9  # 50 GPa (탄성 계수)
# CABLE_DIAMETER = 0.02 # 20mm
# CABLE_AREA = np.pi * (CABLE_DIAMETER / 2)**2
# CABLE_STIFFNESS = (E_MODULUS * CABLE_AREA) # EA 값

# # 목표 설정
# start_x = 2.0
# target_distance = 18.0
# final_x = start_x + target_distance

# # 시뮬레이션 설정
# dt = 0.01
# max_simulation_time = 40.0

# # ---------------------------------------------------------
# # 2. 제어 게인 튜닝
# # ---------------------------------------------------------
# Kp_pos = 2.5
# Kd_pos = 2.0

# Kp_cable = 5.0
# Kd_cable = 2.5
# Kf_cable = 0.95

# MAX_ACCEL = 1.5  # m/s² (고가속)
# MAX_VEL = 3.0    # m/s
# MAX_CABLE_DIFF = 0.6 

# # ---------------------------------------------------------
# # 3. 경로 생성 (S-Curve)
# # ---------------------------------------------------------
# def generate_s_curve_profile(distance, max_v, max_a):
#     t_acc = max_v / max_a
#     d_acc = 0.5 * max_a * t_acc**2
    
#     if d_acc * 2 > distance:
#         d_acc = distance / 2
#         t_acc = np.sqrt(2 * d_acc / max_a)
#         max_v = max_a * t_acc
#         t_dec = t_acc
#         t_cruise = 0
#     else:
#         d_cruise = distance - 2 * d_acc
#         t_cruise = d_cruise / max_v
#         t_dec = t_acc
        
#     total_time = t_acc + t_cruise + t_dec
#     return t_acc, t_cruise, t_dec, max_v, max_a

# p_ta, p_tc, p_td, p_v, p_a = generate_s_curve_profile(target_distance, MAX_VEL, MAX_ACCEL)

# def get_reference(t):
#     if t < p_ta:
#         ref_a = p_a
#         ref_v = p_a * t
#         ref_x = start_x + 0.5 * p_a * t**2
#         phase = "ACCEL"
#     elif t < p_ta + p_tc:
#         dt_cruise = t - p_ta
#         dist_acc = 0.5 * p_a * p_ta**2
#         ref_a = 0
#         ref_v = p_v
#         ref_x = start_x + dist_acc + p_v * dt_cruise
#         phase = "CRUISE"
#     elif t < p_ta + p_tc + p_td:
#         dt_dec = t - (p_ta + p_tc)
#         dist_acc = 0.5 * p_a * p_ta**2
#         dist_cruise = p_v * p_tc
#         ref_a = -p_a
#         ref_v = p_v - p_a * dt_dec
#         ref_x = start_x + dist_acc + dist_cruise + (p_v * dt_dec - 0.5 * p_a * dt_dec**2)
#         phase = "DECEL"
#     else:
#         ref_a = 0
#         ref_v = 0
#         ref_x = final_x
#         phase = "STOP"
#     return ref_x, ref_v, ref_a, phase

# # ---------------------------------------------------------
# # 4. 상태 변수 초기화
# # ---------------------------------------------------------
# crane_x = start_x
# crane_v = 0.0
# theta = 0.0
# omega = 0.0

# # 제어 명령에 의한 길이 (Commanded Length)
# L_left_cmd = L_NOMINAL
# L_right_cmd = L_NOMINAL
# cable_diff = 0.0

# # 실제 물리적 길이 (Actual Length with Stretch)
# L_left_actual = L_NOMINAL
# L_right_actual = L_NOMINAL
# stretch_l = 0.0
# stretch_r = 0.0

# T_left = 0.0
# T_right = 0.0

# time_elapsed = 0.0
# applied_accel = 0.0

# history = {'time':[], 'pos':[], 'vel':[], 'theta':[], 'accel':[], 'diff':[], 'Tl':[], 'Tr':[], 'stretch':[]}

# # ---------------------------------------------------------
# # 메인 루프
# # ---------------------------------------------------------
# running = True
# print(f"=== Elastic Cable Crane Simulation ===")
# print(f"Accel: {MAX_ACCEL} m/s^2")
# print(f"Cable Stiffness (EA): {CABLE_STIFFNESS/1e6:.1f} MN")

# while running:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False
#         if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
#             running = False

#     if time_elapsed < max_simulation_time:
#         # [Step 1] Reference
#         ref_x, ref_v, ref_a, phase = get_reference(time_elapsed)
#         error_pos = ref_x - crane_x
#         error_vel = ref_v - crane_v
        
#         # [Step 2] Trolley Motion
#         trolley_accel_cmd = ref_a + (Kp_pos * error_pos) + (Kd_pos * error_vel)
#         trolley_accel_cmd = np.clip(trolley_accel_cmd, -MAX_ACCEL * 1.5, MAX_ACCEL * 1.5)
        
#         # [Step 3] Active Cable Control (Nominal Length)
#         ff_term = (ref_a / g) * CABLE_SEP * Kf_cable
#         fb_term = -(Kp_cable * theta) - (Kd_cable * omega)
        
#         target_diff = ff_term + fb_term
#         target_diff = np.clip(target_diff, -MAX_CABLE_DIFF, MAX_CABLE_DIFF)
        
#         cable_diff = target_diff
#         L_left_cmd = L_NOMINAL + cable_diff / 2
#         L_right_cmd = L_NOMINAL - cable_diff / 2
        
#         # [Step 4] Tension & Elastic Stretch Calculation (핵심 추가 부분)
#         # 관성력을 포함한 총 하중
#         total_load = m * np.sqrt(g**2 + applied_accel**2)
        
#         # 하중 분배 (케이블 길이에 반비례하여 하중 집중)
#         load_bias = cable_diff / CABLE_SEP 
#         T_left = (total_load / 2) * (1 - load_bias * 2.5)
#         T_right = (total_load / 2) * (1 + load_bias * 2.5)
        
#         T_left = max(100.0, T_left) # 최소 장력 유지
#         T_right = max(100.0, T_right)

#         # Hooke's Law: Delta_L = (T * L) / (E * A)
#         # 길이가 길수록 더 많이 늘어남
#         stretch_l = (T_left * L_left_cmd) / CABLE_STIFFNESS
#         stretch_r = (T_right * L_right_cmd) / CABLE_STIFFNESS
        
#         # 실제 물리적 길이 업데이트
#         L_left_actual = L_left_cmd + stretch_l
#         L_right_actual = L_right_cmd + stretch_r

#         # [Step 5] Physics Update (실제 길이 사용)
#         applied_accel = trolley_accel_cmd
#         crane_v += applied_accel * dt
#         crane_x += crane_v * dt
        
#         # 진자 운동 시 실제 늘어난 길이를 반영하여 물리 계산
#         phi = np.arctan((L_left_actual - L_right_actual) / CABLE_SEP)
#         L_avg_actual = (L_left_actual + L_right_actual) / 2
        
#         alpha = (-g * np.sin(theta) - applied_accel * np.cos(theta) + g * np.sin(phi)) / L_avg_actual
#         alpha -= 0.1 * omega
        
#         omega += alpha * dt
#         theta += omega * dt
        
#         time_elapsed += dt
        
#         # 기록
#         history['time'].append(time_elapsed)
#         history['pos'].append(crane_x)
#         history['vel'].append(crane_v)
#         history['theta'].append(theta)
#         history['accel'].append(applied_accel)
#         history['diff'].append(cable_diff)
#         history['Tl'].append(T_left)
#         history['Tr'].append(T_right)
#         history['stretch'].append((stretch_l + stretch_r)/2 * 1000) # 평균 연신율 (mm)

#     # --- 화면 그리기 ---
#     screen.fill(WHITE)
#     pygame.draw.line(screen, BLACK, (0, HEIGHT-50), (WIDTH, HEIGHT-50), 2)
    
#     center_offset = 100
#     draw_crane_x = int(crane_x * SCALE) + center_offset
#     draw_crane_y = 200
    
#     # 트롤리
#     pygame.draw.rect(screen, DARK_GRAY, (draw_crane_x - 60, draw_crane_y - 30, 120, 30))
#     pygame.draw.circle(screen, BLACK, (draw_crane_x - 40, draw_crane_y - 30), 10)
#     pygame.draw.circle(screen, BLACK, (draw_crane_x + 40, draw_crane_y - 30), 10)
#     pygame.draw.line(screen, GRAY, (0, draw_crane_y), (WIDTH, draw_crane_y), 3)
    
#     att_l_x = draw_crane_x - int(CABLE_SEP * SCALE / 2)
#     att_r_x = draw_crane_x + int(CABLE_SEP * SCALE / 2)
#     att_y = draw_crane_y
    
#     # 컨테이너 위치 (늘어난 길이 L_actual 반영)
#     L_avg_draw = (L_left_actual + L_right_actual) / 2
#     cont_x = draw_crane_x + int(L_avg_draw * SCALE * np.sin(theta))
#     cont_y = draw_crane_y + int(L_avg_draw * SCALE * np.cos(theta))
    
#     # 케이블
#     c_color = RED if abs(cable_diff) > 0.01 else BLACK
#     pygame.draw.line(screen, c_color, (att_l_x, att_y), (cont_x - 20, cont_y - 20), 2)
#     pygame.draw.line(screen, c_color, (att_r_x, att_y), (cont_x + 20, cont_y - 20), 2)
    
#     # 장력 및 늘어난 길이 텍스트
#     # 왼쪽
#     info_l = [f"{int(T_left)} N", f"+{stretch_l*1000:.1f}mm"]
#     for idx, txt in enumerate(info_l):
#         surf = tiny_font.render(txt, True, BLUE)
#         screen.blit(surf, (att_l_x - 80, att_y + 40 + idx*15))
        
#     # 오른쪽
#     info_r = [f"{int(T_right)} N", f"+{stretch_r*1000:.1f}mm"]
#     for idx, txt in enumerate(info_r):
#         surf = tiny_font.render(txt, True, PURPLE)
#         screen.blit(surf, (att_r_x + 10, att_y + 40 + idx*15))

#     # 컨테이너
#     is_safe = abs(theta) < 0.05
#     cont_color = GREEN if is_safe else ORANGE
#     if abs(theta) > 0.05: cont_color = RED
    
#     pygame.draw.rect(screen, cont_color, (cont_x - 30, cont_y - 20, 60, 40))
#     pygame.draw.rect(screen, BLACK, (cont_x - 30, cont_y - 20, 60, 40), 2)
    
#     # 목표 위치
#     target_draw_x = int(final_x * SCALE) + center_offset
#     pygame.draw.line(screen, BLUE, (target_draw_x, draw_crane_y), (target_draw_x, HEIGHT-50), 1)
    
#     # 정보 텍스트
#     infos = [
#         f"Elastic Cable Simulation (Hooke's Law)",
#         f"Phase: {phase}",
#         f"Time: {time_elapsed:.2f} s",
#         f"Cable Diff (Cmd): {cable_diff*1000:.1f} mm",
#         f"Avg Stretch: {(stretch_l+stretch_r)/2*1000:.2f} mm",
#         f"Sway: {theta:.4f} rad",
#         f"Accel: {applied_accel:.2f} m/s²"
#     ]
    
#     for i, txt in enumerate(infos):
#         color = BLACK
#         if "Sway" in txt and not is_safe: color = RED
#         surf = font.render(txt, True, color)
#         screen.blit(surf, (20, 20 + i * 25))

#     # 그래프 1: Sway
#     g_x, g_y, g_w, g_h = 50, 600, 400, 120
#     pygame.draw.rect(screen, (245, 245, 245), (g_x, g_y, g_w, g_h))
#     pygame.draw.rect(screen, BLACK, (g_x, g_y, g_w, g_h), 1)
#     screen.blit(tiny_font.render("Sway Angle (rad)", True, BLACK), (g_x+5, g_y+5))
    
#     limit_y_top = g_y + g_h/2 - (0.05 * 500)
#     limit_y_bot = g_y + g_h/2 - (-0.05 * 500)
#     pygame.draw.line(screen, RED, (g_x, limit_y_top), (g_x+g_w, limit_y_top), 1)
#     pygame.draw.line(screen, RED, (g_x, limit_y_bot), (g_x+g_w, limit_y_bot), 1)

#     if len(history['theta']) > 1:
#         pts = []
#         for i in range(len(history['theta'])):
#             px = g_x + (history['time'][i] / max_simulation_time) * g_w
#             py = g_y + g_h/2 - (history['theta'][i] * 500)
#             pts.append((px, py))
#             if px > g_x + g_w: break
#         if len(pts) > 1:
#             pygame.draw.lines(screen, BLUE, False, pts, 2)

#     # 그래프 2: Cable Diff
#     g_y2 = g_y + 140
#     pygame.draw.rect(screen, (245, 245, 245), (g_x, g_y2, g_w, g_h))
#     pygame.draw.rect(screen, BLACK, (g_x, g_y2, g_w, g_h), 1)
#     screen.blit(tiny_font.render("Cable Diff (m)", True, BLACK), (g_x+5, g_y2+5))
    
#     if len(history['diff']) > 1:
#         pts = []
#         for i in range(len(history['diff'])):
#             px = g_x + (history['time'][i] / max_simulation_time) * g_w
#             py = g_y2 + g_h/2 - (history['diff'][i] * 100)
#             pts.append((px, py))
#             if px > g_x + g_w: break
#         if len(pts) > 1:
#             pygame.draw.lines(screen, RED, False, pts, 2)

#     pygame.display.flip()
#     clock.tick(60)

# pygame.quit()

# # 결과 그래프 저장
# plt.figure(figsize=(12, 10))

# plt.subplot(2, 2, 1)
# plt.plot(history['time'], history['accel'], 'g', label='Trolley Accel')
# plt.ylabel('Accel (m/s^2)')
# plt.title(f'1. Trolley Acceleration (Max: {MAX_ACCEL})')
# plt.grid(True)

# plt.subplot(2, 2, 2)
# plt.plot(history['time'], history['stretch'], 'orange', label='Avg Stretch')
# plt.ylabel('Cable Stretch (mm)')
# plt.title('2. Elastic Stretch (Due to Tension)')
# plt.grid(True)

# plt.subplot(2, 2, 3)
# plt.plot(history['time'], history['theta'], 'b')
# plt.axhline(y=0.05, color='r', linestyle='--', label='Limit (0.05)')
# plt.axhline(y=-0.05, color='r', linestyle='--')
# plt.ylabel('Sway (rad)')
# plt.title('3. Sway Angle Result')
# plt.legend()
# plt.grid(True)

# plt.subplot(2, 2, 4)
# plt.plot(history['time'], history['Tl'], color='blue', label='Left Tension', alpha=0.7)
# plt.plot(history['time'], history['Tr'], color='purple', label='Right Tension', alpha=0.7)
# plt.ylabel('Tension (N)')
# plt.title('4. Cable Tensions')
# plt.legend()
# plt.grid(True)

# plt.tight_layout()
# plt.savefig('elastic_crane_control.png')
# print("Simulation Finished. Graph saved.")


















































































































# import pygame
# import numpy as np
# import sys
# import matplotlib.pyplot as plt

# # ---------------------------------------------------------
# # 초기화 및 설정
# # ---------------------------------------------------------
# pygame.init()

# WIDTH, HEIGHT = 1400, 900
# screen = pygame.display.set_mode((WIDTH, HEIGHT))
# pygame.display.set_caption("Crane Control: Active Winch + Elastic Stretch Visualization")
# clock = pygame.time.Clock()

# # 색상 정의
# WHITE = (255, 255, 255)
# BLACK = (0, 0, 0)
# RED = (220, 50, 50)
# BLUE = (50, 100, 220)
# GRAY = (180, 180, 180)
# GREEN = (0, 180, 0)
# DARK_GRAY = (60, 60, 60)
# ORANGE = (255, 140, 0)
# PURPLE = (128, 0, 128)

# # 폰트 설정
# try:
#     font = pygame.font.SysFont("arial", 22)
#     tiny_font = pygame.font.SysFont("arial", 14)
#     bold_font = pygame.font.SysFont("arial", 18, bold=True)
# except:
#     font = pygame.font.Font(None, 24)
#     tiny_font = pygame.font.Font(None, 16)
#     bold_font = pygame.font.Font(None, 20)

# # ---------------------------------------------------------
# # 1. 물리 및 시스템 상수
# # ---------------------------------------------------------
# g = 9.81
# L_NOMINAL = 4.0   # 기본 케이블 길이 (m)
# m = 1000.0        # 질량 (kg)
# SCALE = 60        # 픽셀/미터 비율
# CABLE_SEP = 1.2   # 트롤리 상단 케이블 간격 (m)

# # 탄성 계수 (효과를 눈으로 확인하기 위해 실제보다 부드럽게 설정)
# E_MODULUS = 40e9  # 40 GPa
# CABLE_DIAMETER = 0.02 # 20mm
# CABLE_AREA = np.pi * (CABLE_DIAMETER / 2)**2
# CABLE_STIFFNESS = (E_MODULUS * CABLE_AREA) 

# # 목표 설정
# start_x = 2.0
# target_distance = 18.0
# final_x = start_x + target_distance

# # 시뮬레이션 설정
# dt = 0.01
# max_simulation_time = 40.0

# # ---------------------------------------------------------
# # 2. 제어 게인
# # ---------------------------------------------------------
# Kp_pos = 2.5
# Kd_pos = 2.0

# Kp_cable = 6.0     # 복원력 (제어용)
# Kd_cable = 3.0     # 댐핑
# Kf_cable = 0.95    # 피드포워드

# MAX_ACCEL = 1.5
# MAX_VEL = 3.0
# MAX_CABLE_DIFF = 0.6 

# # ---------------------------------------------------------
# # 3. 경로 생성 (S-Curve)
# # ---------------------------------------------------------
# def generate_s_curve_profile(distance, max_v, max_a):
#     t_acc = max_v / max_a
#     d_acc = 0.5 * max_a * t_acc**2
#     if d_acc * 2 > distance:
#         d_acc = distance / 2
#         t_acc = np.sqrt(2 * d_acc / max_a)
#         max_v = max_a * t_acc
#         t_dec = t_acc
#         t_cruise = 0
#     else:
#         d_cruise = distance - 2 * d_acc
#         t_cruise = d_cruise / max_v
#         t_dec = t_acc
#     total_time = t_acc + t_cruise + t_dec
#     return t_acc, t_cruise, t_dec, max_v, max_a

# p_ta, p_tc, p_td, p_v, p_a = generate_s_curve_profile(target_distance, MAX_VEL, MAX_ACCEL)

# def get_reference(t):
#     if t < p_ta:
#         ref_a = p_a; ref_v = p_a * t; ref_x = start_x + 0.5 * p_a * t**2
#         phase = "ACCEL"
#     elif t < p_ta + p_tc:
#         dt_c = t - p_ta
#         dist_a = 0.5 * p_a * p_ta**2
#         ref_a = 0; ref_v = p_v; ref_x = start_x + dist_a + p_v * dt_c
#         phase = "CRUISE"
#     elif t < p_ta + p_tc + p_td:
#         dt_d = t - (p_ta + p_tc)
#         dist_a = 0.5 * p_a * p_ta**2; dist_c = p_v * p_tc
#         ref_a = -p_a; ref_v = p_v - p_a * dt_d
#         ref_x = start_x + dist_a + dist_c + (p_v * dt_d - 0.5 * p_a * dt_d**2)
#         phase = "DECEL"
#     else:
#         ref_a = 0; ref_v = 0; ref_x = final_x
#         phase = "STOP"
#     return ref_x, ref_v, ref_a, phase

# # ---------------------------------------------------------
# # 4. 상태 변수 초기화
# # ---------------------------------------------------------
# crane_x = start_x
# crane_v = 0.0
# theta = 0.0
# omega = 0.0

# # 모터가 제어하는 길이 (Commanded Length)
# L_left_cmd = L_NOMINAL
# L_right_cmd = L_NOMINAL

# # 장력에 의해 늘어난 길이 (Stretch)
# stretch_l = 0.0
# stretch_r = 0.0

# # 최종 실제 길이 (Actual)
# L_left_actual = L_NOMINAL
# L_right_actual = L_NOMINAL

# cable_diff = 0.0
# T_left = 0.0
# T_right = 0.0
# time_elapsed = 0.0
# applied_accel = 0.0

# history = {'time':[], 'theta':[], 'accel':[], 'diff':[], 'Tl':[], 'Tr':[], 'stretch':[]}

# # ---------------------------------------------------------
# # 메인 루프
# # ---------------------------------------------------------
# running = True
# print("=== Crane Control: Active Winch + Elastic Stretch Visualization ===")

# while running:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT: running = False
#         if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE: running = False

#     if time_elapsed < max_simulation_time:
#         # [Step 1] Reference
#         ref_x, ref_v, ref_a, phase = get_reference(time_elapsed)
#         error_pos = ref_x - crane_x
#         error_vel = ref_v - crane_v
        
#         # [Step 2] Trolley Motion
#         trolley_accel_cmd = ref_a + (Kp_pos * error_pos) + (Kd_pos * error_vel)
#         trolley_accel_cmd = np.clip(trolley_accel_cmd, -MAX_ACCEL * 1.5, MAX_ACCEL * 1.5)
        
#         # [Step 3] Active Cable Control (모터 제어)
#         # 피드포워드: 가속도(a)가 0이면 이 항은 0이 됨
#         ff_term = (ref_a / g) * CABLE_SEP * Kf_cable
#         # 피드백: 흔들림(theta)이 0이면 이 항은 0이 됨
#         fb_term = -(Kp_cable * theta) - (Kd_cable * omega)
        
#         # 목표 차이값 계산
#         target_diff = ff_term + fb_term
        
#         # [중요] 장력이 같아지고(=가속도 0), 흔들림이 없으면(=theta 0)
#         # target_diff는 자연스럽게 0이 됩니다. (Active Return to 0)
        
#         target_diff = np.clip(target_diff, -MAX_CABLE_DIFF, MAX_CABLE_DIFF)
#         cable_diff = target_diff
        
#         # 모터 명령 길이 (Winch)
#         L_left_cmd = L_NOMINAL + cable_diff / 2
#         L_right_cmd = L_NOMINAL - cable_diff / 2
        
#         # [Step 4] Tension & Elasticity (물리 현상)
#         total_load = m * np.sqrt(g**2 + applied_accel**2)
#         load_bias = cable_diff / CABLE_SEP 
        
#         T_left = (total_load / 2) * (1 - load_bias * 2.5)
#         T_right = (total_load / 2) * (1 + load_bias * 2.5)
        
#         # 최소 장력 보정
#         T_left = max(100.0, T_left); T_right = max(100.0, T_right)

#         # 늘어난 길이 계산 (Hooke's Law)
#         stretch_l = (T_left * L_left_cmd) / CABLE_STIFFNESS
#         stretch_r = (T_right * L_right_cmd) / CABLE_STIFFNESS
        
#         # 실제 길이 업데이트
#         L_left_actual = L_left_cmd + stretch_l
#         L_right_actual = L_right_cmd + stretch_r

#         # [Step 5] Physics Update
#         applied_accel = trolley_accel_cmd
#         crane_v += applied_accel * dt
#         crane_x += crane_v * dt
        
#         # 실제 길이를 반영한 진자 운동
#         phi = np.arctan((L_left_actual - L_right_actual) / CABLE_SEP)
#         L_avg_actual = (L_left_actual + L_right_actual) / 2
        
#         alpha = (-g * np.sin(theta) - applied_accel * np.cos(theta) + g * np.sin(phi)) / L_avg_actual
#         alpha -= 0.1 * omega
        
#         omega += alpha * dt
#         theta += omega * dt
#         time_elapsed += dt
        
#         # 기록
#         history['time'].append(time_elapsed)
#         history['theta'].append(theta)
#         history['accel'].append(applied_accel)
#         history['diff'].append(cable_diff)
#         history['Tl'].append(T_left)
#         history['Tr'].append(T_right)
#         history['stretch'].append((stretch_l + stretch_r)/2 * 1000)

#     # --- 화면 그리기 ---
#     screen.fill(WHITE)
#     pygame.draw.line(screen, BLACK, (0, HEIGHT-50), (WIDTH, HEIGHT-50), 2)
    
#     center_offset = 100
#     draw_crane_x = int(crane_x * SCALE) + center_offset
#     draw_crane_y = 200
    
#     # 트롤리
#     pygame.draw.rect(screen, DARK_GRAY, (draw_crane_x - 60, draw_crane_y - 30, 120, 30))
#     pygame.draw.line(screen, GRAY, (0, draw_crane_y), (WIDTH, draw_crane_y), 3)
    
#     att_l_x = draw_crane_x - int(CABLE_SEP * SCALE / 2)
#     att_r_x = draw_crane_x + int(CABLE_SEP * SCALE / 2)
#     att_y = draw_crane_y
    
#     L_draw = (L_left_actual + L_right_actual) / 2
#     cont_x = draw_crane_x + int(L_draw * SCALE * np.sin(theta))
#     cont_y = draw_crane_y + int(L_draw * SCALE * np.cos(theta))
    
#     # 케이블
#     c_color = RED if abs(cable_diff) > 0.01 else BLACK
#     pygame.draw.line(screen, c_color, (att_l_x, att_y), (cont_x - 20, cont_y - 20), 2)
#     pygame.draw.line(screen, c_color, (att_r_x, att_y), (cont_x + 20, cont_y - 20), 2)
    
#     # ---------------------------------------------------------
#     # [상세 정보 시각화] 케이블 길이 변화 내역
#     # ---------------------------------------------------------
#     # 왼쪽 케이블 정보
#     l_active_color = RED if abs(cable_diff) > 0.01 else GREEN
#     text_y = att_y + 40
    
#     # 장력 표시
#     screen.blit(bold_font.render(f"T: {int(T_left)} N", True, BLUE), (att_l_x - 120, text_y))
#     # 모터 제어량 (Active)
#     l_adj = (L_left_cmd - L_NOMINAL) * 1000
#     screen.blit(tiny_font.render(f"Motor: {l_adj:+.1f}mm", True, l_active_color), (att_l_x - 120, text_y + 20))
#     # 탄성 변형량 (Passive)
#     screen.blit(tiny_font.render(f"Stretch: +{stretch_l*1000:.1f}mm", True, GRAY), (att_l_x - 120, text_y + 35))
    
#     # 오른쪽 케이블 정보
#     screen.blit(bold_font.render(f"T: {int(T_right)} N", True, PURPLE), (att_r_x + 10, text_y))
#     r_adj = (L_right_cmd - L_NOMINAL) * 1000
#     screen.blit(tiny_font.render(f"Motor: {r_adj:+.1f}mm", True, l_active_color), (att_r_x + 10, text_y + 20))
#     screen.blit(tiny_font.render(f"Stretch: +{stretch_r*1000:.1f}mm", True, GRAY), (att_r_x + 10, text_y + 35))

#     # 컨테이너
#     is_safe = abs(theta) < 0.05
#     cont_color = GREEN if is_safe else ORANGE
#     if abs(theta) > 0.05: cont_color = RED
    
#     pygame.draw.rect(screen, cont_color, (cont_x - 30, cont_y - 20, 60, 40))
#     pygame.draw.rect(screen, BLACK, (cont_x - 30, cont_y - 20, 60, 40), 2)
    
#     # 목표 위치
#     target_draw_x = int(final_x * SCALE) + center_offset
#     pygame.draw.line(screen, BLUE, (target_draw_x, draw_crane_y), (target_draw_x, HEIGHT-50), 1)
    
#     # 메인 정보 패널
#     infos = [
#         f"Control Status: {'Active Adjusting' if abs(cable_diff) > 0.01 else 'Stable (Returned to 0)'}",
#         f"Phase: {phase}",
#         f"Time: {time_elapsed:.2f} s",
#         f"Active Diff (Motor): {cable_diff*1000:.1f} mm",
#         f"Sway: {theta:.4f} rad",
#         f"Accel: {applied_accel:.2f} m/s²"
#     ]
    
#     for i, txt in enumerate(infos):
#         c = RED if "Active" in txt and abs(cable_diff) > 0.01 else BLACK
#         surf = font.render(txt, True, c)
#         screen.blit(surf, (20, 20 + i * 30))

#     # 그래프 1: Sway
#     g_x, g_y, g_w, g_h = 50, 600, 300, 100
#     pygame.draw.rect(screen, (245, 245, 245), (g_x, g_y, g_w, g_h))
#     pygame.draw.rect(screen, BLACK, (g_x, g_y, g_w, g_h), 1)
#     screen.blit(tiny_font.render("Sway (rad)", True, BLACK), (g_x+5, g_y+5))
#     if len(history['theta']) > 1:
#         pts = []
#         for i in range(len(history['theta'])):
#             px = g_x + (history['time'][i] / max_simulation_time) * g_w
#             py = g_y + g_h/2 - (history['theta'][i] * 500)
#             pts.append((px, py))
#             if px > g_x + g_w: break
#         if len(pts) > 1: pygame.draw.lines(screen, BLUE, False, pts, 2)

#     # 그래프 2: Active Diff (Motor)
#     g_x += 320
#     pygame.draw.rect(screen, (245, 245, 245), (g_x, g_y, g_w, g_h))
#     pygame.draw.rect(screen, BLACK, (g_x, g_y, g_w, g_h), 1)
#     screen.blit(tiny_font.render("Motor Control Diff (m)", True, BLACK), (g_x+5, g_y+5))
#     if len(history['diff']) > 1:
#         pts = []
#         for i in range(len(history['diff'])):
#             px = g_x + (history['time'][i] / max_simulation_time) * g_w
#             py = g_y + g_h/2 - (history['diff'][i] * 100)
#             pts.append((px, py))
#             if px > g_x + g_w: break
#         if len(pts) > 1: pygame.draw.lines(screen, RED, False, pts, 2)

#     # 그래프 3: Tension
#     g_x += 320
#     pygame.draw.rect(screen, (245, 245, 245), (g_x, g_y, g_w, g_h))
#     pygame.draw.rect(screen, BLACK, (g_x, g_y, g_w, g_h), 1)
#     screen.blit(tiny_font.render("Tension (N) L:Blue R:Purple", True, BLACK), (g_x+5, g_y+5))
#     if len(history['Tl']) > 1:
#         pts_l, pts_r = [], []
#         max_t = 8000
#         for i in range(len(history['Tl'])):
#             px = g_x + (history['time'][i] / max_simulation_time) * g_w
#             py_l = g_y + g_h - (history['Tl'][i] / max_t) * g_h
#             py_r = g_y + g_h - (history['Tr'][i] / max_t) * g_h
#             pts_l.append((px, py_l)); pts_r.append((px, py_r))
#             if px > g_x + g_w: break
#         if len(pts_l) > 1:
#             pygame.draw.lines(screen, BLUE, False, pts_l, 1)
#             pygame.draw.lines(screen, PURPLE, False, pts_r, 1)

#     pygame.display.flip()
#     clock.tick(60)

# pygame.quit()























































































































































# import pygame
# import numpy as np
# import sys
# import matplotlib.pyplot as plt

# # ---------------------------------------------------------
# # 초기화 및 설정
# # ---------------------------------------------------------
# pygame.init()

# WIDTH, HEIGHT = 1400, 900
# screen = pygame.display.set_mode((WIDTH, HEIGHT))
# pygame.display.set_caption("Crane Control: Visible Cable Length Change (Exaggerated View)")
# clock = pygame.time.Clock()

# # 색상 정의
# WHITE = (255, 255, 255)
# BLACK = (0, 0, 0)
# RED = (220, 50, 50)     # 줄어듦 (Pull)
# BLUE = (50, 100, 220)   # 늘어남 (Release)
# GRAY = (180, 180, 180)
# GREEN = (0, 180, 0)
# DARK_GRAY = (60, 60, 60)
# ORANGE = (255, 140, 0)
# PURPLE = (128, 0, 128)

# # 폰트 설정
# try:
#     font = pygame.font.SysFont("arial", 22)
#     tiny_font = pygame.font.SysFont("arial", 14)
#     bold_font = pygame.font.SysFont("arial", 18, bold=True)
# except:
#     font = pygame.font.Font(None, 24)
#     tiny_font = pygame.font.Font(None, 16)
#     bold_font = pygame.font.Font(None, 20)

# # ---------------------------------------------------------
# # 1. 물리 및 시스템 상수
# # ---------------------------------------------------------
# g = 9.81
# L_NOMINAL = 4.0   # 기본 케이블 길이 (m)
# m = 1000.0        # 질량 (kg)
# SCALE = 60        # 픽셀/미터 비율
# CABLE_SEP = 1.2   # 트롤리 상단 케이블 간격 (m)

# # 탄성 계수
# E_MODULUS = 40e9 
# CABLE_DIAMETER = 0.02
# CABLE_AREA = np.pi * (CABLE_DIAMETER / 2)**2
# CABLE_STIFFNESS = (E_MODULUS * CABLE_AREA) 

# # [중요] 시각적 과장 계수 (실제 물리보다 20배 더 크게 보여줌)
# VISUAL_EXAGGERATION = 20.0 

# # 목표 설정
# start_x = 2.0
# target_distance = 18.0
# final_x = start_x + target_distance

# # 시뮬레이션 설정
# dt = 0.01
# max_simulation_time = 40.0

# # ---------------------------------------------------------
# # 2. 제어 게인
# # ---------------------------------------------------------
# Kp_pos = 2.5
# Kd_pos = 2.0

# Kp_cable = 6.0
# Kd_cable = 3.0
# Kf_cable = 0.95 

# MAX_ACCEL = 1.5
# MAX_VEL = 3.0
# MAX_CABLE_DIFF = 0.6 

# # ---------------------------------------------------------
# # 3. 경로 생성 (S-Curve)
# # ---------------------------------------------------------
# def generate_s_curve_profile(distance, max_v, max_a):
#     t_acc = max_v / max_a
#     d_acc = 0.5 * max_a * t_acc**2
#     if d_acc * 2 > distance:
#         d_acc = distance / 2
#         t_acc = np.sqrt(2 * d_acc / max_a)
#         max_v = max_a * t_acc
#         t_dec = t_acc
#         t_cruise = 0
#     else:
#         d_cruise = distance - 2 * d_acc
#         t_cruise = d_cruise / max_v
#         t_dec = t_acc
#     return t_acc, t_cruise, t_dec, max_v, max_a

# p_ta, p_tc, p_td, p_v, p_a = generate_s_curve_profile(target_distance, MAX_VEL, MAX_ACCEL)

# def get_reference(t):
#     if t < p_ta:
#         ref_a = p_a; ref_v = p_a * t; ref_x = start_x + 0.5 * p_a * t**2
#         phase = "ACCEL"
#     elif t < p_ta + p_tc:
#         dt_c = t - p_ta
#         dist_a = 0.5 * p_a * p_ta**2
#         ref_a = 0; ref_v = p_v; ref_x = start_x + dist_a + p_v * dt_c
#         phase = "CRUISE"
#     elif t < p_ta + p_tc + p_td:
#         dt_d = t - (p_ta + p_tc)
#         dist_a = 0.5 * p_a * p_ta**2; dist_c = p_v * p_tc
#         ref_a = -p_a; ref_v = p_v - p_a * dt_d
#         ref_x = start_x + dist_a + dist_c + (p_v * dt_d - 0.5 * p_a * dt_d**2)
#         phase = "DECEL"
#     else:
#         ref_a = 0; ref_v = 0; ref_x = final_x
#         phase = "STOP"
#     return ref_x, ref_v, ref_a, phase

# # ---------------------------------------------------------
# # 4. 상태 변수 초기화
# # ---------------------------------------------------------
# crane_x = start_x
# crane_v = 0.0
# theta = 0.0
# omega = 0.0

# L_left_cmd = L_NOMINAL
# L_right_cmd = L_NOMINAL
# L_left_actual = L_NOMINAL
# L_right_actual = L_NOMINAL

# stretch_l = 0.0
# stretch_r = 0.0
# cable_diff = 0.0
# T_left = 0.0
# T_right = 0.0
# time_elapsed = 0.0
# applied_accel = 0.0

# history = {'time':[], 'theta':[], 'accel':[], 'diff':[], 'Tl':[], 'Tr':[], 'stretch':[]}

# # ---------------------------------------------------------
# # 메인 루프
# # ---------------------------------------------------------
# running = True
# print("=== Crane Control: Exaggerated Visuals ===")
# print(f"Visual Exaggeration Factor: {VISUAL_EXAGGERATION}x")

# while running:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT: running = False
#         if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE: running = False

#     if time_elapsed < max_simulation_time:
#         # [Step 1] Reference
#         ref_x, ref_v, ref_a, phase = get_reference(time_elapsed)
#         error_pos = ref_x - crane_x
#         error_vel = ref_v - crane_v
        
#         # [Step 2] Trolley Motion
#         trolley_accel_cmd = ref_a + (Kp_pos * error_pos) + (Kd_pos * error_vel)
#         trolley_accel_cmd = np.clip(trolley_accel_cmd, -MAX_ACCEL * 1.5, MAX_ACCEL * 1.5)
        
#         # [Step 3] Active Cable Control
#         ff_term = (ref_a / g) * CABLE_SEP * Kf_cable
#         fb_term = -(Kp_cable * theta) - (Kd_cable * omega)
        
#         target_diff = ff_term + fb_term
#         target_diff = np.clip(target_diff, -MAX_CABLE_DIFF, MAX_CABLE_DIFF)
#         cable_diff = target_diff
        
#         L_left_cmd = L_NOMINAL + cable_diff / 2
#         L_right_cmd = L_NOMINAL - cable_diff / 2
        
#         # [Step 4] Tension & Elasticity
#         total_load = m * np.sqrt(g**2 + applied_accel**2)
#         load_bias = cable_diff / CABLE_SEP 
        
#         T_left = (total_load / 2) * (1 - load_bias * 2.5)
#         T_right = (total_load / 2) * (1 + load_bias * 2.5)
        
#         T_left = max(100.0, T_left); T_right = max(100.0, T_right)

#         stretch_l = (T_left * L_left_cmd) / CABLE_STIFFNESS
#         stretch_r = (T_right * L_right_cmd) / CABLE_STIFFNESS
        
#         L_left_actual = L_left_cmd + stretch_l
#         L_right_actual = L_right_cmd + stretch_r

#         # [Step 5] Physics Update
#         applied_accel = trolley_accel_cmd
#         crane_v += applied_accel * dt
#         crane_x += crane_v * dt
        
#         phi = np.arctan((L_left_actual - L_right_actual) / CABLE_SEP)
#         L_avg_actual = (L_left_actual + L_right_actual) / 2
        
#         alpha = (-g * np.sin(theta) - applied_accel * np.cos(theta) + g * np.sin(phi)) / L_avg_actual
#         alpha -= 0.1 * omega
        
#         omega += alpha * dt
#         theta += omega * dt
#         time_elapsed += dt
        
#         history['time'].append(time_elapsed)
#         history['theta'].append(theta)
#         history['accel'].append(applied_accel)
#         history['diff'].append(cable_diff)
#         history['Tl'].append(T_left)
#         history['Tr'].append(T_right)
#         history['stretch'].append((stretch_l + stretch_r)/2 * 1000)

#     # --- 화면 그리기 ---
#     screen.fill(WHITE)
#     pygame.draw.line(screen, BLACK, (0, HEIGHT-50), (WIDTH, HEIGHT-50), 2)
    
#     center_offset = 100
#     draw_crane_x = int(crane_x * SCALE) + center_offset
#     draw_crane_y = 200
    
#     # 트롤리
#     pygame.draw.rect(screen, DARK_GRAY, (draw_crane_x - 60, draw_crane_y - 30, 120, 30))
#     pygame.draw.line(screen, GRAY, (0, draw_crane_y), (WIDTH, draw_crane_y), 3)
    
#     att_l_x = draw_crane_x - int(CABLE_SEP * SCALE / 2)
#     att_r_x = draw_crane_x + int(CABLE_SEP * SCALE / 2)
#     att_y = draw_crane_y
    
#     # -------------------------------------------------------------
#     # [시각화 핵심] 변경된 길이를 20배 과장해서 그리기
#     # -------------------------------------------------------------
#     # 실제 길이 변화량
#     delta_L_left = L_left_actual - L_NOMINAL
#     delta_L_right = L_right_actual - L_NOMINAL
    
#     # 과장된 시각적 길이 (그리기용)
#     L_vis_left = L_NOMINAL + delta_L_left * VISUAL_EXAGGERATION
#     L_vis_right = L_NOMINAL + delta_L_right * VISUAL_EXAGGERATION
    
#     # 기하학적 계산 (과장된 길이 기반)
#     # 두 원(케이블)의 교점 찾기 근사 (비대칭)
#     # 시각적 비대칭 각도
#     vis_tilt = np.arctan((L_vis_left - L_vis_right) / CABLE_SEP)
#     L_vis_avg = (L_vis_left + L_vis_right) / 2
    
#     # 컨테이너 좌표 (과장된 좌표)
#     cont_x = draw_crane_x + int(L_vis_avg * SCALE * np.sin(theta))
#     cont_y = draw_crane_y + int(L_vis_avg * SCALE * np.cos(theta))
    
#     # 왼쪽 케이블 (줄어들면 빨강, 늘어나면 파랑)
#     color_l = BLACK
#     if delta_L_left < -0.001: color_l = RED    # 당김 (짧아짐)
#     elif delta_L_left > 0.001: color_l = BLUE  # 풀림 (길어짐)
#     pygame.draw.line(screen, color_l, (att_l_x, att_y), (cont_x - 20, cont_y - 20), 4) # 두껍게

#     # 오른쪽 케이블
#     color_r = BLACK
#     if delta_L_right < -0.001: color_r = RED
#     elif delta_L_right > 0.001: color_r = BLUE
#     pygame.draw.line(screen, color_r, (att_r_x, att_y), (cont_x + 20, cont_y - 20), 4) # 두껍게
    
#     # -------------------------------------------------------------
#     # [윈치 게이지] 모터 작동 상태 바 그래프
#     # -------------------------------------------------------------
#     gauge_w, gauge_h = 20, 100
#     gauge_y = att_y - 120
    
#     # 왼쪽 게이지
#     pygame.draw.rect(screen, GRAY, (att_l_x - 10, gauge_y, gauge_w, gauge_h), 1)
#     pygame.draw.line(screen, BLACK, (att_l_x - 10, gauge_y + gauge_h/2), (att_l_x + 10, gauge_y + gauge_h/2), 1) # 0점
    
#     bar_h_l = int(-delta_L_left * 200) # 길이 변화에 비례
#     if bar_h_l > 0: # 당김 (위로)
#         pygame.draw.rect(screen, RED, (att_l_x - 10, gauge_y + gauge_h/2 - bar_h_l, gauge_w, bar_h_l))
#     else: # 풀림 (아래로)
#         pygame.draw.rect(screen, BLUE, (att_l_x - 10, gauge_y + gauge_h/2, gauge_w, -bar_h_l))
        
#     # 오른쪽 게이지
#     pygame.draw.rect(screen, GRAY, (att_r_x - 10, gauge_y, gauge_w, gauge_h), 1)
#     pygame.draw.line(screen, BLACK, (att_r_x - 10, gauge_y + gauge_h/2), (att_r_x + 10, gauge_y + gauge_h/2), 1)
    
#     bar_h_r = int(-delta_L_right * 200)
#     if bar_h_r > 0:
#         pygame.draw.rect(screen, RED, (att_r_x - 10, gauge_y + gauge_h/2 - bar_h_r, gauge_w, bar_h_r))
#     else:
#         pygame.draw.rect(screen, BLUE, (att_r_x - 10, gauge_y + gauge_h/2, gauge_w, -bar_h_r))
        
#     screen.blit(tiny_font.render("WINCH", True, BLACK), (draw_crane_x - 20, gauge_y - 20))

#     # 컨테이너
#     is_safe = abs(theta) < 0.05
#     cont_color = GREEN if is_safe else ORANGE
#     if abs(theta) > 0.05: cont_color = RED
    
#     pygame.draw.rect(screen, cont_color, (cont_x - 30, cont_y - 20, 60, 40))
#     pygame.draw.rect(screen, BLACK, (cont_x - 30, cont_y - 20, 60, 40), 2)
    
#     # 정보 텍스트 (실제 물리값 표시)
#     infos = [
#         f"Visual Exaggeration: x{int(VISUAL_EXAGGERATION)}",
#         f"L_Left: {L_left_actual:.3f}m ({'PULL' if delta_L_left<0 else 'RELEASE'})",
#         f"L_Right: {L_right_actual:.3f}m ({'PULL' if delta_L_right<0 else 'RELEASE'})",
#         f"Sway: {theta:.4f} rad",
#     ]
    
#     for i, txt in enumerate(infos):
#         screen.blit(font.render(txt, True, BLACK), (20, 20 + i * 25))

#     # 하단 그래프들 (생략 없이 그리기)
#     g_x, g_y, g_w, g_h = 50, 600, 300, 100
#     pygame.draw.rect(screen, (245, 245, 245), (g_x, g_y, g_w, g_h))
#     pygame.draw.rect(screen, BLACK, (g_x, g_y, g_w, g_h), 1)
#     screen.blit(tiny_font.render("Sway (rad)", True, BLACK), (g_x+5, g_y+5))
#     if len(history['theta']) > 1:
#         pts = []
#         for i in range(len(history['theta'])):
#             px = g_x + (history['time'][i] / max_simulation_time) * g_w
#             py = g_y + g_h/2 - (history['theta'][i] * 500)
#             pts.append((px, py)); 
#             if px > g_x + g_w: break
#         if len(pts) > 1: pygame.draw.lines(screen, BLUE, False, pts, 2)

#     g_x += 320
#     pygame.draw.rect(screen, (245, 245, 245), (g_x, g_y, g_w, g_h))
#     pygame.draw.rect(screen, BLACK, (g_x, g_y, g_w, g_h), 1)
#     screen.blit(tiny_font.render("Cable Diff (m)", True, BLACK), (g_x+5, g_y+5))
#     if len(history['diff']) > 1:
#         pts = []
#         for i in range(len(history['diff'])):
#             px = g_x + (history['time'][i] / max_simulation_time) * g_w
#             py = g_y + g_h/2 - (history['diff'][i] * 100)
#             pts.append((px, py)); 
#             if px > g_x + g_w: break
#         if len(pts) > 1: pygame.draw.lines(screen, RED, False, pts, 2)

#     pygame.display.flip()
#     clock.tick(60)

# pygame.quit()


































































































































































# import pygame
# import numpy as np
# import sys
# import matplotlib.pyplot as plt

# # ---------------------------------------------------------
# # 초기화 및 설정
# # ---------------------------------------------------------
# pygame.init()

# WIDTH, HEIGHT = 1400, 900
# screen = pygame.display.set_mode((WIDTH, HEIGHT))
# pygame.display.set_caption("Hybrid Crane Control with Elastic Cables")
# clock = pygame.time.Clock()

# # 색상 정의
# WHITE = (255, 255, 255)
# BLACK = (0, 0, 0)
# RED = (200, 50, 50)
# BLUE = (50, 100, 200)
# GRAY = (180, 180, 180)
# GREEN = (0, 150, 0)
# DARK_GRAY = (60, 60, 60)
# ORANGE = (255, 140, 0)
# PURPLE = (128, 0, 128)

# # 폰트 설정
# try:
#     font = pygame.font.SysFont("arial", 24)
#     tiny_font = pygame.font.SysFont("arial", 16)
#     bold_font = pygame.font.SysFont("arial", 20, bold=True)
# except:
#     font = pygame.font.Font(None, 24)
#     tiny_font = pygame.font.Font(None, 16)
#     bold_font = pygame.font.Font(None, 20)

# # ---------------------------------------------------------
# # 1. 물리 및 시스템 상수
# # ---------------------------------------------------------
# g = 9.81
# L_NOMINAL = 4.0   # 기본 케이블 길이 (m)
# m = 1000.0        # 질량 (kg)
# SCALE = 60        # 픽셀/미터 비율
# CABLE_SEP = 1.2   # 트롤리 상단 케이블 간격 (m)

# # [NEW] 케이블 탄성 계수 (N/m) - 시각적 확인을 위해 실제 강철보다 낮게 설정
# # 값이 낮을수록 장력에 의해 많이 늘어납니다.
# CABLE_K = 15000.0 

# # 목표 설정
# start_x = 2.0
# target_distance = 18.0
# final_x = start_x + target_distance

# # 시뮬레이션 설정
# dt = 0.01
# max_simulation_time = 40.0

# # ---------------------------------------------------------
# # 2. 제어 게인 튜닝
# # ---------------------------------------------------------
# Kp_pos = 2.5
# Kd_pos = 2.0

# Kp_cable = 5.0
# Kd_cable = 2.5
# Kf_cable = 0.95

# MAX_ACCEL = 0.8
# MAX_VEL = 2.5
# MAX_CABLE_DIFF = 0.5

# # ---------------------------------------------------------
# # 3. 경로 생성 (S-Curve)
# # ---------------------------------------------------------
# def generate_s_curve_profile(distance, max_v, max_a):
#     t_acc = max_v / max_a
#     d_acc = 0.5 * max_a * t_acc**2
    
#     if d_acc * 2 > distance:
#         d_acc = distance / 2
#         t_acc = np.sqrt(2 * d_acc / max_a)
#         max_v = max_a * t_acc
#         t_dec = t_acc
#         t_cruise = 0
#     else:
#         d_cruise = distance - 2 * d_acc
#         t_cruise = d_cruise / max_v
#         t_dec = t_acc
        
#     total_time = t_acc + t_cruise + t_dec
#     return t_acc, t_cruise, t_dec, max_v, max_a

# p_ta, p_tc, p_td, p_v, p_a = generate_s_curve_profile(target_distance, MAX_VEL, MAX_ACCEL)

# def get_reference(t):
#     if t < p_ta:
#         ref_a = p_a
#         ref_v = p_a * t
#         ref_x = start_x + 0.5 * p_a * t**2
#         phase = "ACCEL"
#     elif t < p_ta + p_tc:
#         dt_cruise = t - p_ta
#         dist_acc = 0.5 * p_a * p_ta**2
#         ref_a = 0
#         ref_v = p_v
#         ref_x = start_x + dist_acc + p_v * dt_cruise
#         phase = "CRUISE"
#     elif t < p_ta + p_tc + p_td:
#         dt_dec = t - (p_ta + p_tc)
#         dist_acc = 0.5 * p_a * p_ta**2
#         dist_cruise = p_v * p_tc
#         ref_a = -p_a
#         ref_v = p_v - p_a * dt_dec
#         ref_x = start_x + dist_acc + dist_cruise + (p_v * dt_dec - 0.5 * p_a * dt_dec**2)
#         phase = "DECEL"
#     else:
#         ref_a = 0
#         ref_v = 0
#         ref_x = final_x
#         phase = "STOP"
#     return ref_x, ref_v, ref_a, phase

# # ---------------------------------------------------------
# # 4. 상태 변수 초기화
# # ---------------------------------------------------------
# crane_x = start_x
# crane_v = 0.0
# theta = 0.0
# omega = 0.0

# # 실제 길이 변수
# L_left = L_NOMINAL
# L_right = L_NOMINAL

# # 제어 명령용 변수
# ctrl_diff = 0.0
# real_diff = 0.0 # 장력에 의해 늘어난 실제 길이 차이

# T_left = 0.0
# T_right = 0.0

# time_elapsed = 0.0
# applied_accel = 0.0

# history = {'time':[], 'pos':[], 'vel':[], 'theta':[], 'accel':[], 'diff':[], 'Tl':[], 'Tr':[]}

# # ---------------------------------------------------------
# # 메인 루프
# # ---------------------------------------------------------
# running = True
# print("=== Hybrid Crane Control Simulation ===")
# print(f"탄성 모드 활성화: Stiffness K={CABLE_K} N/m")

# while running:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False
#         if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
#             running = False

#     if time_elapsed < max_simulation_time:
#         # [Step 1] Reference
#         ref_x, ref_v, ref_a, phase = get_reference(time_elapsed)
#         error_pos = ref_x - crane_x
#         error_vel = ref_v - crane_v
        
#         # [Step 2] Trolley Motion
#         trolley_accel_cmd = ref_a + (Kp_pos * error_pos) + (Kd_pos * error_vel)
#         trolley_accel_cmd = np.clip(trolley_accel_cmd, -MAX_ACCEL * 1.5, MAX_ACCEL * 1.5)
        
#         # [Step 3] Active Cable Control (Command)
#         ff_term = (ref_a / g) * CABLE_SEP * Kf_cable
#         fb_term = -(Kp_cable * theta) - (Kd_cable * omega)
        
#         target_diff = ff_term + fb_term
#         target_diff = np.clip(target_diff, -MAX_CABLE_DIFF, MAX_CABLE_DIFF)
        
#         # 제어기가 명령한 '기본 길이' (장력 고려 전)
#         ctrl_diff = target_diff
#         L_left_cmd = L_NOMINAL + ctrl_diff / 2
#         L_right_cmd = L_NOMINAL - ctrl_diff / 2
        
#         # [Step 4] Tension Calculation (장력 추정)
#         # 하중 분배: 제어 명령상의 길이 차이를 기준으로 대략적인 하중 분배 계산
#         total_load = m * np.sqrt(g**2 + applied_accel**2)
#         load_bias = ctrl_diff / CABLE_SEP 
        
#         T_left = (total_load / 2) * (1 - load_bias * 2.5)
#         T_right = (total_load / 2) * (1 + load_bias * 2.5)
        
#         T_left = max(100.0, T_left)
#         T_right = max(100.0, T_right)

#         # [Step 5] Apply Elasticity (탄성 적용 - 요청사항)
#         # Hooke's Law: Delta_L = Force / Stiffness
#         stretch_left = T_left / CABLE_K
#         stretch_right = T_right / CABLE_K
        
#         # 실제 길이는 명령 길이 + 늘어난 길이
#         L_left = L_left_cmd + stretch_left
#         L_right = L_right_cmd + stretch_right
        
#         # 실제 길이 차이 업데이트 (물리 계산용)
#         real_diff = L_left - L_right

#         # [Step 6] Physics Update
#         applied_accel = trolley_accel_cmd
#         crane_v += applied_accel * dt
#         crane_x += crane_v * dt
        
#         # 실제 늘어난 길이를 반영하여 물리 계산
#         # 길이가 다르면 컨테이너가 기울어지는 효과(phi)
#         phi = np.arctan(real_diff / CABLE_SEP)
#         L_avg = (L_left + L_right) / 2
        
#         alpha = (-g * np.sin(theta) - applied_accel * np.cos(theta) + g * np.sin(phi)) / L_avg
#         alpha -= 0.1 * omega # Damping
        
#         omega += alpha * dt
#         theta += omega * dt
        
#         time_elapsed += dt
        
#         # 기록
#         history['time'].append(time_elapsed)
#         history['pos'].append(crane_x)
#         history['vel'].append(crane_v)
#         history['theta'].append(theta)
#         history['accel'].append(applied_accel)
#         history['diff'].append(real_diff) # 실제 길이 차이 기록
#         history['Tl'].append(T_left)
#         history['Tr'].append(T_right)

#     # --- 화면 그리기 ---
#     screen.fill(WHITE)
#     pygame.draw.line(screen, BLACK, (0, HEIGHT-50), (WIDTH, HEIGHT-50), 2)
    
#     center_offset = 100
#     draw_crane_x = int(crane_x * SCALE) + center_offset
#     draw_crane_y = 200
    
#     # 트롤리
#     pygame.draw.rect(screen, DARK_GRAY, (draw_crane_x - 60, draw_crane_y - 30, 120, 30))
#     pygame.draw.circle(screen, BLACK, (draw_crane_x - 40, draw_crane_y - 30), 10)
#     pygame.draw.circle(screen, BLACK, (draw_crane_x + 40, draw_crane_y - 30), 10)
#     pygame.draw.line(screen, GRAY, (0, draw_crane_y), (WIDTH, draw_crane_y), 3)
    
#     att_l_x = draw_crane_x - int(CABLE_SEP * SCALE / 2)
#     att_r_x = draw_crane_x + int(CABLE_SEP * SCALE / 2)
#     att_y = draw_crane_y
    
#     # 컨테이너 위치 계산 (늘어난 길이 L_avg 반영)
#     L_avg_pixel = L_avg * SCALE
#     cont_x = draw_crane_x + int(L_avg_pixel * np.sin(theta))
#     cont_y = draw_crane_y + int(L_avg_pixel * np.cos(theta))
    
#     # 케이블 그리기
#     # 장력 차이에 의한 길이 변화를 시각적으로 보여줌
#     c_color = RED if abs(real_diff) > 0.05 else BLACK
#     pygame.draw.line(screen, c_color, (att_l_x, att_y), (cont_x - 20, cont_y - 20), 2)
#     pygame.draw.line(screen, c_color, (att_r_x, att_y), (cont_x + 20, cont_y - 20), 2)
    
#     # --- 장력 텍스트 표시 ---
#     tl_text = bold_font.render(f"{int(T_left)} N", True, BLUE)
#     tr_text = bold_font.render(f"{int(T_right)} N", True, PURPLE)
    
#     # 길이 텍스트 추가 표시
#     l_len_text = tiny_font.render(f"L: {L_left:.2f}m", True, BLUE)
#     r_len_text = tiny_font.render(f"L: {L_right:.2f}m", True, PURPLE)

#     screen.blit(tl_text, (att_l_x - 80, att_y + 40))
#     screen.blit(l_len_text, (att_l_x - 80, att_y + 60))
    
#     screen.blit(tr_text, (att_r_x + 20, att_y + 40))
#     screen.blit(r_len_text, (att_r_x + 20, att_y + 60))
    
#     # 컨테이너
#     is_safe = abs(theta) < 0.05
#     cont_color = GREEN if is_safe else ORANGE
#     if abs(theta) > 0.05: cont_color = RED
    
#     pygame.draw.rect(screen, cont_color, (cont_x - 30, cont_y - 20, 60, 40))
#     pygame.draw.rect(screen, BLACK, (cont_x - 30, cont_y - 20, 60, 40), 2)
    
#     # 목표 위치
#     target_draw_x = int(final_x * SCALE) + center_offset
#     pygame.draw.line(screen, BLUE, (target_draw_x, draw_crane_y), (target_draw_x, HEIGHT-50), 1)
    
#     # 정보 텍스트
#     infos = [
#         f"Elastic Cable Simulation (K={CABLE_K})",
#         f"Phase: {phase}",
#         f"Time: {time_elapsed:.2f} s",
#         f"Real Diff: {real_diff*1000:.1f} mm",
#         f"Stretch L: {(L_left - L_left_cmd)*1000:.1f} mm", # 늘어난 길이
#         f"Stretch R: {(L_right - L_right_cmd)*1000:.1f} mm",
#         f"Sway: {theta:.4f} rad"
#     ]
    
#     for i, txt in enumerate(infos):
#         color = BLACK
#         surf = font.render(txt, True, color)
#         screen.blit(surf, (20, 20 + i * 25))

#     # 그래프 1: Sway
#     g_x, g_y, g_w, g_h = 50, 600, 400, 120
#     pygame.draw.rect(screen, (245, 245, 245), (g_x, g_y, g_w, g_h))
#     pygame.draw.rect(screen, BLACK, (g_x, g_y, g_w, g_h), 1)
#     screen.blit(tiny_font.render("Sway Angle (rad)", True, BLACK), (g_x+5, g_y+5))
    
#     if len(history['theta']) > 1:
#         pts = []
#         for i in range(len(history['theta'])):
#             px = g_x + (history['time'][i] / max_simulation_time) * g_w
#             py = g_y + g_h/2 - (history['theta'][i] * 500)
#             pts.append((px, py))
#             if px > g_x + g_w: break
#         if len(pts) > 1:
#             pygame.draw.lines(screen, BLUE, False, pts, 2)

#     # 그래프 2: Tension
#     g_y2 = g_y + 140
#     pygame.draw.rect(screen, (245, 245, 245), (g_x, g_y2, g_w, g_h))
#     pygame.draw.rect(screen, BLACK, (g_x, g_y2, g_w, g_h), 1)
#     screen.blit(tiny_font.render("Tensions (N)", True, BLACK), (g_x+5, g_y2+5))
    
#     if len(history['Tl']) > 1:
#         pts_l = []
#         pts_r = []
#         max_t = max(max(history['Tl']), max(history['Tr'])) + 500
#         min_t = min(min(history['Tl']), min(history['Tr'])) - 500
#         t_range = max_t - min_t if max_t != min_t else 1.0
        
#         for i in range(len(history['Tl'])):
#             px = g_x + (history['time'][i] / max_simulation_time) * g_w
#             py_l = g_y2 + g_h - ((history['Tl'][i] - min_t) / t_range) * g_h
#             py_r = g_y2 + g_h - ((history['Tr'][i] - min_t) / t_range) * g_h
#             pts_l.append((px, py_l))
#             pts_r.append((px, py_r))
#             if px > g_x + g_w: break
            
#         if len(pts_l) > 1:
#             pygame.draw.lines(screen, BLUE, False, pts_l, 1)
#             pygame.draw.lines(screen, PURPLE, False, pts_r, 1)

#     pygame.display.flip()
#     clock.tick(60)

# pygame.quit()

# # 결과 그래프 저장
# plt.figure(figsize=(12, 10))

# plt.subplot(2, 2, 1)
# plt.plot(history['time'], history['accel'], 'g', label='Trolley Accel')
# plt.ylabel('Accel (m/s^2)')
# plt.title('1. Trolley Acceleration')
# plt.grid(True)

# plt.subplot(2, 2, 2)
# plt.plot(history['time'], history['diff'], 'r', label='Real Length Diff')
# plt.ylabel('Cable Length Diff (m)')
# plt.title('2. Actual Cable Difference (Elastic)')
# plt.grid(True)

# plt.subplot(2, 2, 3)
# plt.plot(history['time'], history['theta'], 'b')
# plt.axhline(y=0.05, color='r', linestyle='--', label='Limit')
# plt.ylabel('Sway (rad)')
# plt.title('3. Sway Angle Result')
# plt.grid(True)

# plt.subplot(2, 2, 4)
# plt.plot(history['time'], history['Tl'], color='blue', label='Left Tension')
# plt.plot(history['time'], history['Tr'], color='purple', label='Right Tension')
# plt.ylabel('Tension (N)')
# plt.title('4. Cable Tensions')
# plt.legend()
# plt.grid(True)

# plt.tight_layout()
# plt.savefig('elastic_crane_simulation.png')
# print("Simulation Finished. Graph saved.")




















































































































import pygame
import numpy as np
import sys
import matplotlib.pyplot as plt

# ---------------------------------------------------------
# 초기화 및 설정
# ---------------------------------------------------------
pygame.init()

WIDTH, HEIGHT = 1400, 900
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Hybrid Crane Control: Anti-Sway Optimized")
clock = pygame.time.Clock()

# 색상 정의
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (200, 50, 50)
BLUE = (50, 100, 200)
GRAY = (180, 180, 180)
GREEN = (0, 150, 0)
DARK_GRAY = (60, 60, 60)
ORANGE = (255, 140, 0)
PURPLE = (128, 0, 128)

# 폰트 설정
try:
    font = pygame.font.SysFont("arial", 24)
    tiny_font = pygame.font.SysFont("arial", 16)
    bold_font = pygame.font.SysFont("arial", 20, bold=True)
except:
    font = pygame.font.Font(None, 24)
    tiny_font = pygame.font.Font(None, 16)
    bold_font = pygame.font.Font(None, 20)

# ---------------------------------------------------------
# 1. 물리 및 시스템 상수
# ---------------------------------------------------------
g = 9.81
L_NOMINAL = 4.0   
m = 1000.0        
SCALE = 60        
CABLE_SEP = 1.2   
CABLE_K = 15000.0 # 탄성 계수

# 목표 설정
start_x = 2.0
target_distance = 18.0
final_x = start_x + target_distance

# 시뮬레이션 설정
dt = 0.01
max_simulation_time = 40.0

# ---------------------------------------------------------
# 2. 제어 게인 튜닝 (Anti-Sway 최적화)
# ---------------------------------------------------------
# 트롤리 제어 게인
Kp_pos = 2.5
Kd_pos = 3.0 # 감쇠를 조금 더 높여 부드럽게 정지

# [수정됨] 케이블 제어 게인 - 탄성을 이기기 위해 게인을 대폭 상향
Kp_cable = 15.0  # 기존 5.0 -> 15.0 (각도에 더 민감하게 반응)
Kd_cable = 8.0   # 기존 2.5 -> 8.0 (흔들림 속도를 강력하게 제동)
Kf_cable = 1.0   # Feedforward

MAX_ACCEL = 0.8
MAX_VEL = 2.5
MAX_CABLE_DIFF = 0.6 # 제어 범위 약간 확장

# ---------------------------------------------------------
# 3. 경로 생성 (S-Curve)
# ---------------------------------------------------------
def generate_s_curve_profile(distance, max_v, max_a):
    t_acc = max_v / max_a
    d_acc = 0.5 * max_a * t_acc**2
    
    if d_acc * 2 > distance:
        d_acc = distance / 2
        t_acc = np.sqrt(2 * d_acc / max_a)
        max_v = max_a * t_acc
        t_dec = t_acc
        t_cruise = 0
    else:
        d_cruise = distance - 2 * d_acc
        t_cruise = d_cruise / max_v
        t_dec = t_acc
        
    total_time = t_acc + t_cruise + t_dec
    return t_acc, t_cruise, t_dec, max_v, max_a

p_ta, p_tc, p_td, p_v, p_a = generate_s_curve_profile(target_distance, MAX_VEL, MAX_ACCEL)

def get_reference(t):
    if t < p_ta:
        ref_a = p_a
        ref_v = p_a * t
        ref_x = start_x + 0.5 * p_a * t**2
        phase = "ACCEL"
    elif t < p_ta + p_tc:
        dt_cruise = t - p_ta
        dist_acc = 0.5 * p_a * p_ta**2
        ref_a = 0
        ref_v = p_v
        ref_x = start_x + dist_acc + p_v * dt_cruise
        phase = "CRUISE"
    elif t < p_ta + p_tc + p_td:
        dt_dec = t - (p_ta + p_tc)
        dist_acc = 0.5 * p_a * p_ta**2
        dist_cruise = p_v * p_tc
        ref_a = -p_a
        ref_v = p_v - p_a * dt_dec
        ref_x = start_x + dist_acc + dist_cruise + (p_v * dt_dec - 0.5 * p_a * dt_dec**2)
        phase = "DECEL"
    else:
        ref_a = 0
        ref_v = 0
        ref_x = final_x
        phase = "STOP"
    return ref_x, ref_v, ref_a, phase

# ---------------------------------------------------------
# 4. 상태 변수 초기화
# ---------------------------------------------------------
crane_x = start_x
crane_v = 0.0
theta = 0.0
omega = 0.0

L_left = L_NOMINAL
L_right = L_NOMINAL

ctrl_diff = 0.0
real_diff = 0.0
stiff_comp_val = 0.0 # 탄성 보상값 기록용

T_left = m * g / 2 # 초기 장력 추정치
T_right = m * g / 2

time_elapsed = 0.0
applied_accel = 0.0

history = {'time':[], 'pos':[], 'vel':[], 'theta':[], 'accel':[], 'diff':[], 'Tl':[], 'Tr':[]}

# ---------------------------------------------------------
# 메인 루프
# ---------------------------------------------------------
running = True
print("=== Anti-Sway Optimized Simulation ===")
print(f"탄성 보상(Stiffness Compensation) 활성화됨.")

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            running = False

    if time_elapsed < max_simulation_time:
        # [Step 1] Reference
        ref_x, ref_v, ref_a, phase = get_reference(time_elapsed)
        error_pos = ref_x - crane_x
        error_vel = ref_v - crane_v
        
        # [Step 2] Trolley Motion
        trolley_accel_cmd = ref_a + (Kp_pos * error_pos) + (Kd_pos * error_vel)
        trolley_accel_cmd = np.clip(trolley_accel_cmd, -MAX_ACCEL * 1.5, MAX_ACCEL * 1.5)
        
        # [Step 3] Active Cable Control (Anti-Sway Logic)
        ff_term = (ref_a / g) * CABLE_SEP * Kf_cable
        
        # [중요 수정] 부호를 변경하여 Positive Feedback 방지 및 Damping 강화
        # Theta > 0 (우측 흔들림) -> fb_term > 0 -> diff > 0
        # diff > 0 이면 L_right = Nom - diff/2 (오른쪽 줄 짧아짐)
        # 오른쪽 줄을 당겨 올려서 복원력을 생성 (Active Damping)
        fb_term = (Kp_cable * theta) + (Kd_cable * omega)
        
        target_diff = ff_term + fb_term

        # [NEW] Stiffness Compensation (탄성 보상)
        # 늘어날 것으로 예상되는 길이 차이를 미리 명령에 반영하여 상쇄
        # (T_left - T_right) 양수 -> 왼쪽이 더 늘어남 -> 왼쪽을 더 감아야 함 -> diff를 줄여야 함?
        # L_left = Nom + diff/2. 왼쪽을 짧게 하려면 diff를 줄여야 함 (음수 방향).
        # 보상 식: stretch_diff = (T_left - T_right) / K
        # 명령 보정: target_diff -= stretch_diff * Gain
        
        stretch_diff_est = (T_left - T_right) / CABLE_K
        stiffness_compensation = stretch_diff_est * 2.0 # 2.0은 보상 게인 (Overcompensate)
        
        # 최종 명령 생성 (기본 제어 + 탄성 보상)
        ctrl_diff = target_diff - stiffness_compensation
        ctrl_diff = np.clip(ctrl_diff, -MAX_CABLE_DIFF, MAX_CABLE_DIFF)
        stiff_comp_val = stiffness_compensation

        L_left_cmd = L_NOMINAL + ctrl_diff / 2
        L_right_cmd = L_NOMINAL - ctrl_diff / 2
        
        # [Step 4] Tension Calculation (장력 업데이트)
        total_load = m * np.sqrt(g**2 + applied_accel**2)
        
        # 장력 분배는 '실제 물리적 위치'에 기반해야 함 (명령값이 아닌)
        # 근사치를 위해 현재의 실제 길이 차이(real_diff) 사용
        load_bias = real_diff / CABLE_SEP 
        
        T_left = (total_load / 2) * (1 - load_bias * 2.5)
        T_right = (total_load / 2) * (1 + load_bias * 2.5)
        
        T_left = max(100.0, T_left)
        T_right = max(100.0, T_right)

        # [Step 5] Apply Elasticity
        stretch_left = T_left / CABLE_K
        stretch_right = T_right / CABLE_K
        
        L_left = L_left_cmd + stretch_left
        L_right = L_right_cmd + stretch_right
        
        real_diff = L_left - L_right

        # [Step 6] Physics Update
        applied_accel = trolley_accel_cmd
        crane_v += applied_accel * dt
        crane_x += crane_v * dt
        
        phi = np.arctan(real_diff / CABLE_SEP)
        L_avg = (L_left + L_right) / 2
        
        alpha = (-g * np.sin(theta) - applied_accel * np.cos(theta) + g * np.sin(phi)) / L_avg
        alpha -= 0.1 * omega 
        
        omega += alpha * dt
        theta += omega * dt
        
        time_elapsed += dt
        
        history['time'].append(time_elapsed)
        history['pos'].append(crane_x)
        history['vel'].append(crane_v)
        history['theta'].append(theta)
        history['accel'].append(applied_accel)
        history['diff'].append(real_diff)
        history['Tl'].append(T_left)
        history['Tr'].append(T_right)

    # --- 화면 그리기 ---
    screen.fill(WHITE)
    pygame.draw.line(screen, BLACK, (0, HEIGHT-50), (WIDTH, HEIGHT-50), 2)
    
    center_offset = 100
    draw_crane_x = int(crane_x * SCALE) + center_offset
    draw_crane_y = 200
    
    pygame.draw.rect(screen, DARK_GRAY, (draw_crane_x - 60, draw_crane_y - 30, 120, 30))
    pygame.draw.circle(screen, BLACK, (draw_crane_x - 40, draw_crane_y - 30), 10)
    pygame.draw.circle(screen, BLACK, (draw_crane_x + 40, draw_crane_y - 30), 10)
    pygame.draw.line(screen, GRAY, (0, draw_crane_y), (WIDTH, draw_crane_y), 3)
    
    att_l_x = draw_crane_x - int(CABLE_SEP * SCALE / 2)
    att_r_x = draw_crane_x + int(CABLE_SEP * SCALE / 2)
    att_y = draw_crane_y
    
    L_avg_pixel = L_avg * SCALE
    cont_x = draw_crane_x + int(L_avg_pixel * np.sin(theta))
    cont_y = draw_crane_y + int(L_avg_pixel * np.cos(theta))
    
    c_color = RED if abs(real_diff) > 0.05 else BLACK
    pygame.draw.line(screen, c_color, (att_l_x, att_y), (cont_x - 20, cont_y - 20), 2)
    pygame.draw.line(screen, c_color, (att_r_x, att_y), (cont_x + 20, cont_y - 20), 2)
    
    tl_text = bold_font.render(f"{int(T_left)} N", True, BLUE)
    tr_text = bold_font.render(f"{int(T_right)} N", True, PURPLE)
    l_len_text = tiny_font.render(f"L: {L_left:.2f}m", True, BLUE)
    r_len_text = tiny_font.render(f"L: {L_right:.2f}m", True, PURPLE)

    screen.blit(tl_text, (att_l_x - 80, att_y + 40))
    screen.blit(l_len_text, (att_l_x - 80, att_y + 60))
    screen.blit(tr_text, (att_r_x + 20, att_y + 40))
    screen.blit(r_len_text, (att_r_x + 20, att_y + 60))
    
    is_safe = abs(theta) < 0.05
    cont_color = GREEN if is_safe else ORANGE
    if abs(theta) > 0.05: cont_color = RED
    
    pygame.draw.rect(screen, cont_color, (cont_x - 30, cont_y - 20, 60, 40))
    pygame.draw.rect(screen, BLACK, (cont_x - 30, cont_y - 20, 60, 40), 2)
    
    target_draw_x = int(final_x * SCALE) + center_offset
    pygame.draw.line(screen, BLUE, (target_draw_x, draw_crane_y), (target_draw_x, HEIGHT-50), 1)
    
    infos = [
        f"Anti-Sway Mode (Elastic K={CABLE_K})",
        f"Phase: {phase}",
        f"Real Diff: {real_diff*1000:.1f} mm",
        f"Compensate: {stiff_comp_val*1000:.1f} mm",
        f"Sway: {theta:.4f} rad"
    ]
    
    for i, txt in enumerate(infos):
        color = BLACK
        if "Sway" in txt and not is_safe: color = RED
        surf = font.render(txt, True, color)
        screen.blit(surf, (20, 20 + i * 25))

    # 그래프 1: Sway
    g_x, g_y, g_w, g_h = 50, 600, 400, 120
    pygame.draw.rect(screen, (245, 245, 245), (g_x, g_y, g_w, g_h))
    pygame.draw.rect(screen, BLACK, (g_x, g_y, g_w, g_h), 1)
    screen.blit(tiny_font.render("Sway Angle (rad)", True, BLACK), (g_x+5, g_y+5))
    
    if len(history['theta']) > 1:
        pts = []
        for i in range(len(history['theta'])):
            px = g_x + (history['time'][i] / max_simulation_time) * g_w
            py = g_y + g_h/2 - (history['theta'][i] * 500)
            pts.append((px, py))
            if px > g_x + g_w: break
        if len(pts) > 1:
            pygame.draw.lines(screen, BLUE, False, pts, 2)

    # 그래프 2: Tension
    g_y2 = g_y + 140
    pygame.draw.rect(screen, (245, 245, 245), (g_x, g_y2, g_w, g_h))
    pygame.draw.rect(screen, BLACK, (g_x, g_y2, g_w, g_h), 1)
    screen.blit(tiny_font.render("Tensions (N)", True, BLACK), (g_x+5, g_y2+5))
    
    if len(history['Tl']) > 1:
        pts_l = []
        pts_r = []
        max_t = max(max(history['Tl']), max(history['Tr'])) + 500
        min_t = min(min(history['Tl']), min(history['Tr'])) - 500
        t_range = max_t - min_t if max_t != min_t else 1.0
        
        for i in range(len(history['Tl'])):
            px = g_x + (history['time'][i] / max_simulation_time) * g_w
            py_l = g_y2 + g_h - ((history['Tl'][i] - min_t) / t_range) * g_h
            py_r = g_y2 + g_h - ((history['Tr'][i] - min_t) / t_range) * g_h
            pts_l.append((px, py_l))
            pts_r.append((px, py_r))
            if px > g_x + g_w: break
            
        if len(pts_l) > 1:
            pygame.draw.lines(screen, BLUE, False, pts_l, 1)
            pygame.draw.lines(screen, PURPLE, False, pts_r, 1)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()

# 결과 그래프 저장
plt.figure(figsize=(12, 10))

plt.subplot(2, 2, 1)
plt.plot(history['time'], history['accel'], 'g', label='Trolley Accel')
plt.ylabel('Accel (m/s^2)')
plt.title('1. Trolley Acceleration')
plt.grid(True)

plt.subplot(2, 2, 2)
plt.plot(history['time'], history['diff'], 'r', label='Real Length Diff')
plt.ylabel('Cable Length Diff (m)')
plt.title('2. Actual Cable Difference (Active Control)')
plt.grid(True)

plt.subplot(2, 2, 3)
plt.plot(history['time'], history['theta'], 'b')
plt.axhline(y=0.05, color='r', linestyle='--', label='Limit')
plt.ylabel('Sway (rad)')
plt.title('3. Sway Angle Result (Anti-Sway)')
plt.grid(True)

plt.subplot(2, 2, 4)
plt.plot(history['time'], history['Tl'], color='blue', label='Left Tension')
plt.plot(history['time'], history['Tr'], color='purple', label='Right Tension')
plt.ylabel('Tension (N)')
plt.title('4. Cable Tensions')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig('elastic_antisway_result.png')
print("Simulation Finished. Anti-sway optimized graph saved.")