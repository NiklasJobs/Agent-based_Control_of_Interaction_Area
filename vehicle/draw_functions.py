import pygame

# colour definition
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

def draw_avoiding_WP(vehicle, screen):
    for point in vehicle.avoiding_WP:
        pygame.draw.circle(screen, BLACK, point, 3)
    
def draw_path(vehicle, screen):
    if len(vehicle.move_points) < 1:
        return
    current_position = (int(vehicle.x), int(vehicle.y))
    points_to_draw = [current_position] + vehicle.move_points
    pygame.draw.lines(screen, (255, 0, 0), False, points_to_draw, 2)    
    
def draw_rover(rover, screen):
    pygame.draw.circle(screen, rover.color, (rover.x, rover.y), rover.radius)
    font = pygame.font.Font(None, 20)
    text_surface = font.render(str(rover.id), True, WHITE)
    screen.blit(text_surface, (rover.x - 5, rover.y - 5))

