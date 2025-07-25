# # 显示得分
# import pygame

# pygame.init()
# screen = pygame.display.set_mode((800, 600))
# font = pygame.font.SysFont(None, 48)
# score = 0
# score_image = font.render(f"Score: {score}", True, (0, 0, 0))

# running = True
# while running:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False
#     screen.fill((230, 230, 230))
#     screen.blit(score_image, (10, 10))
#     pygame.display.flip()

# pygame.quit()