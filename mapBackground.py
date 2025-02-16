import pygame


class Background(pygame.sprite.Sprite):
    def __init__(self, image, location, scale):
        pygame.sprite.Sprite.__init__(self)
        self.image = pygame.image.load(image)
        self.image = pygame.transform.rotozoom(self.image, 0, scale)
        self.rect = self.image.get_rect()
        self.rect.left, self.rect.top = location

class Button:
    def __init__(self, screen, text, position, size, color, font, font_size, border_color=(0, 0, 0), click_color=None):
        self.font = font
        self.screen = screen
        self.text = text
        self.position = position
        self.size = size
        self.color = color
        self.font_size = font_size
        self.border_color = border_color
        self.click_color = click_color if click_color else color
        self.rect = pygame.Rect(0, 0, size[0], size[1])
        self.rect.center = position
        self.clicked = False

    def draw(self):
        if self.clicked:
            current_color = self.click_color
        else:
            current_color = self.color

        font = pygame.font.SysFont(self.font, self.font_size)
        text_surface = font.render(self.text, True, (0, 0, 0))
        text_rect = text_surface.get_rect(center=self.position)

        pygame.draw.rect(self.screen, self.border_color, self.rect)
        pygame.draw.rect(self.screen, current_color, self.rect.inflate(-4, -4))
        self.screen.blit(text_surface, text_rect)

    def is_clicked(self, mouse_pos):
        isClicked = self.rect.collidepoint(mouse_pos)
        return isClicked

    def toggle_color(self):
        self.clicked = not self.clicked
        self.is_hovered = False  # Reset hover status when clicked

    def set_clicked(self, clicked):
        self.clicked = clicked