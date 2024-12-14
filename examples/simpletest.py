import pygame
from pyphys.objects import Body
from pyphys.engine import PhysicsEngine

pygame.init()
screen = pygame.display.set_mode((800, 600))
clock = pygame.time.Clock()

engine = PhysicsEngine()
player = Body(100, 100, 50, 50)
ground = Body(0, 500, 800, 50, mass=float('inf'))  # Infinite mass = immovable

engine.add_body(player)
engine.add_body(ground)

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((0, 0, 0))
    pygame.draw.rect(screen, (255, 0, 0), (player.x, player.y, player.width, player.height))
    pygame.draw.rect(screen, (0, 255, 0), (ground.x, ground.y, ground.width, ground.height))

    engine.update(1 / 60)  # Fixed timestep
    pygame.display.flip()
    clock.tick(60)

pygame.quit()
