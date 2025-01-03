import sys
import pygame
from pyphys.engine import PhysicsEngine
from pyphys.integration import Sprite, SpriteBinder
from pyphys.objects import Circle, Polygon, Rectangle

WIDTH, HEIGHT = 800, 600
PLAYER_SPEED_X = PLAYER_SPEED_Y = 3
FPS = 60
GRAVITY = 9.8
RESISTANCE = 0.1
COLORS = {
    "white": (255, 255, 255),
    "red": (255, 0, 0),
    "green": (0, 255, 0),
    "blue": (0, 0, 255),
    "black": (0, 0, 0),
    "orange": (255, 128, 0),
    "cyan": (0, 255, 255),
    "night": (43, 38, 74)
}

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("2D Physics Engine")
sprite_binder = SpriteBinder(HEIGHT)

# Dynamic Bodies
mario_sprite_image = pygame.image.load('mario.png')  # Load the image
mario_sprite = Sprite(mario_sprite_image)
player_rect = Rectangle(
    x=450,
    y=250,
    width=mario_sprite.rect.width,
    height=mario_sprite.rect.height,
    name="Player",
    mass=50,
)
sprite_binder.bind(mario_sprite, player_rect, HEIGHT)

box1 = Rectangle(
    x=100,
    y=300,
    width=50,
    height=50,
    mass=10
)

box2 = Rectangle(
    x=300,
    y=300,
    width=50,
    height=50,
    mass=300
)

# Static Bodies
house_1 = Polygon(
    x=200,
    y=50,
    vertices=[(0, 0), (0, 50), (20, 60), (40, 50), (40, 0)],
    is_static=True
)

ground = Rectangle(
    x=WIDTH / 2,
    y=10,
    width=WIDTH,
    height=20,
    is_static=True,
    dynamic_friction=0.5,
    static_friction=0.7,
    bounce=0.7,
    name="Ground",
)

PhysicsWorld = PhysicsEngine(
    [
        player_rect,
        house_1,
        ground,
        box1,
        box2
    ],
    GRAVITY,
    RESISTANCE
)

move_left = False
move_right = False
move_up = False
move_down = False
dt = 1 / FPS  

# Main game loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                move_left = True
            elif event.key == pygame.K_RIGHT:
                move_right = True
            elif event.key == pygame.K_UP:
                move_up = True
            elif event.key == pygame.K_DOWN:
                move_down = True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_LEFT:
                move_left = False
            elif event.key == pygame.K_RIGHT:
                move_right = False
            elif event.key == pygame.K_UP:
                move_up = False
            elif event.key == pygame.K_DOWN:
                move_down = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            new_star = Circle(
                x=mouse_x,
                y=HEIGHT - mouse_y,
                radius = 5,
                mass=3
            )
            PhysicsWorld.add(new_star)

    if move_left:
        player_rect.velocity[0] -= PLAYER_SPEED_X
    if move_right:
        player_rect.velocity[0] += PLAYER_SPEED_X
    if move_up:
        player_rect.velocity[1] += PLAYER_SPEED_Y
    if move_down:
        player_rect.velocity[1] -= PLAYER_SPEED_Y

    PhysicsWorld.step(dt)
    sprite_binder.update()
    screen.fill(COLORS["night"])

    for body in PhysicsWorld.bodies:
        if body.name == "Player":
            continue
        if body.shape_type == "Polygon":
            color = COLORS["black"]
            pygame.draw.polygon(
                screen,
                color,
                [(vertex.x, HEIGHT - vertex.y) for vertex in body.get_vertices()],
            )
        elif body.shape_type == "Circle":
            color = COLORS["white"]
            pygame.draw.circle(
                screen, color, (body.center[0], HEIGHT - body.center[1]), body.radius
            )

    screen.blit(mario_sprite.image, mario_sprite.rect)
            
    for point in PhysicsWorld._contact_points:
        pygame.draw.circle(screen, COLORS["green"], (point.x, HEIGHT - point.y), 4)

    pygame.display.flip()

    pygame.time.Clock().tick(FPS)
