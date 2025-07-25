# 绘制外星人
import pygame

pygame.init()
screen = pygame.display.set_mode((800, 600))
alien_image = pygame.Surface((50, 50))  # 模拟外星人
alien_image.fill((0, 255, 0))  # 绿色方块

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    screen.fill((230, 230, 230))
    screen.blit(alien_image, (375, 275))  # 居中绘制
    pygame.display.flip()

pygame.quit()