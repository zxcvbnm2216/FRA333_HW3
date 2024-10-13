# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
1.ปวริศร์_6536
2.ภาสวร_6548
'''
#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here
import pygame # type: ignore
import numpy as np # type: ignore
from FRA333_HW3_36_48 import endEffectorJacobianHW3, checkSingularityHW3, computeEffortHW3

pygame.init()
screen = pygame.display.set_mode((1280, 720))
pygame.display.set_caption('Robotic Arm Control')

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)
RED = (255, 0, 0)

running = True
q = [0.0, 0.0, 0.0]  # Initial joint angles for 3 joints
w = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Initial external wrench components
clock = pygame.time.Clock()

while running:
    screen.fill(WHITE)
    keys = pygame.key.get_pressed()

    # Control joint angles
    if keys[pygame.K_a]:
        q[0] += 0.01  # Increase joint 1 angle
    if keys[pygame.K_q]:
        q[0] -= 0.01  # Decrease joint 1 angle
    if keys[pygame.K_w]:
        q[1] += 0.01  # Increase joint 2 angle
    if keys[pygame.K_s]:
        q[1] -= 0.01  # Decrease joint 2 angle
    if keys[pygame.K_e]:
        q[2] += 0.01  # Increase joint 3 angle
    if keys[pygame.K_d]:
        q[2] -= 0.01  # Decrease joint 3 angle

    # Control wrench components
    if keys[pygame.K_i]:
        w[0] += 0.1  # Increase force component Fx
    if keys[pygame.K_k]:
        w[0] -= 0.1  # Decrease force component Fx
    if keys[pygame.K_j]:
        w[1] += 0.1  # Increase force component Fy
    if keys[pygame.K_l]:
        w[1] -= 0.1  # Decrease force component Fy
    if keys[pygame.K_u]:
        w[2] += 0.1  # Increase force component Fz
    if keys[pygame.K_o]:
        w[2] -= 0.1  # Decrease force component Fz
    if keys[pygame.K_r]:
        w[3] += 0.1  # Increase moment component Mx
    if keys[pygame.K_f]:
        w[3] -= 0.1  # Decrease moment component Mx
    if keys[pygame.K_t]:
        w[4] += 0.1  # Increase moment component My
    if keys[pygame.K_g]:
        w[4] -= 0.1  # Decrease moment component My
    if keys[pygame.K_y]:
        w[5] += 0.1  # Increase moment component Mz
    if keys[pygame.K_h]:
        w[5] -= 0.1  # Decrease moment component Mz

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Display Jacobian, Singularity, Joint Angles, and Effort Information
    jacobian = endEffectorJacobianHW3(q)  # Using all 3 joint angles for Jacobian calculation
    singularity = checkSingularityHW3(q)
    efforts = computeEffortHW3(q, w)

    font = pygame.font.SysFont(None, 24)
    y_offset = 50

    # Display Joint Angles
    joint_angles_text = font.render('Joint Angles:', True, BLACK)
    screen.blit(joint_angles_text, (50, y_offset))
    y_offset += 30
    for i, angle in enumerate(q):
        angle_text = font.render(f'Joint {i + 1}: {angle:.2f} rad', True, BLACK)
        screen.blit(angle_text, (50, y_offset))
        y_offset += 30

    # Display Wrench Components
    wrench_text = font.render('Wrench Components (Fx, Fy, Fz, Mx, My, Mz):', True, BLACK)
    screen.blit(wrench_text, (50, y_offset))
    y_offset += 30
    wrench_values_text = font.render(f'{w}', True, BLACK)
    screen.blit(wrench_values_text, (50, y_offset))
    y_offset += 30

    # Display Jacobian as a matrix
    jacobian_text = font.render('Jacobian:', True, BLACK)
    screen.blit(jacobian_text, (50, y_offset))
    y_offset += 30
    for row in jacobian:
        row_text = font.render(' '.join(f'{value:.2f}' for value in row), True, BLACK)
        screen.blit(row_text, (50, y_offset))
        y_offset += 30

    # Display Singularity Status
    singularity_text = font.render(f'Singularity: {singularity}', True, RED if singularity else BLUE)
    screen.blit(singularity_text, (50, y_offset))
    y_offset += 30

    # Display Efforts
    efforts_text = font.render('Efforts:', True, BLACK)
    screen.blit(efforts_text, (50, y_offset))
    y_offset += 30
    for i, effort in enumerate(efforts):
        effort_text = font.render(f'Joint {i + 1}: {effort:.2f}', True, BLACK)
        screen.blit(effort_text, (50, y_offset))
        y_offset += 30

    pygame.display.flip()
    clock.tick(60)  # Limit the loop to 60 iterations per second

pygame.quit()
#==============================================================================================================#