import pygame

# 初始化 Pygame
pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("Alien Invasion")

# 加载飞船图像
try:
    ship = pygame.image.load("images/ship.bmp")  # 尝试加载 .bmp
    if not ship:
        raise FileNotFoundError("Image file is empty or invalid")
except FileNotFoundError:
    print("Error: 'images/ship.bmp' not found. Please add the file to the images folder.")
    pygame.quit()
    exit()

ship_rect = ship.get_rect()
ship_rect.centerx = 400  # 初始位置：屏幕中心
ship_rect.bottom = 580   # 靠近底部
ship_speed = 5

# 游戏主循环
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    # 键盘控制
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT] and ship_rect.left > 0:
        ship_rect.x -= ship_speed
    if keys[pygame.K_RIGHT] and ship_rect.right < 800:
        ship_rect.x += ship_speed
    # 绘制
    screen.fill((230, 230, 230))  # 填充背景色
    screen.blit(ship, ship_rect)   # 绘制飞船
    pygame.display.flip()

# 退出 Pygame
pygame.quit()