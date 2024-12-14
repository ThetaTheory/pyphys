# Sprite and Object Sync

import math
import pygame
from pyphys.objects import Body

class Sprite(pygame.sprite.Sprite): 
    def __init__(self, image=None, color=None, height=None, width=None): 
        super().__init__() 

        if image:
            self.image = image  # Directly set the image if passed
        elif color and height and width:
            # If no image is provided, create a colored surface
            self.image = pygame.Surface([width, height])  
            pygame.draw.rect(self.image, color, pygame.Rect(0, 0, width, height))  
        else:
            raise ValueError("Either 'image' or 'color', 'height', and 'width' must be provided.")
  
        self.original_image = self.image.copy() 
        self.rect = self.image.get_rect() 

class SpriteBinder:
    def __init__(self, screen_height: int):
        self.bindings = []
        self.screen_height = screen_height

    def bind(self, sprite: Sprite, body: Body, screen_height: int):
        """Bind a Pygame sprite to a physics Body."""
        if not isinstance(sprite, Sprite):
            raise TypeError("The sprite must be an instance of the Sprite class or its subclass.")
        self.bindings.append((sprite, body, screen_height))

    def update(self):
        """Synchronize sprite positions and angles with their corresponding physics bodies."""
        for sprite, body, screen_height in self.bindings:
            # Synchronize position (using body center)
            sprite.rect.center = (body.center.x, screen_height - body.center.y)
            
            # Rotate the sprite image based on the body's angle
            rotated_image = pygame.transform.rotate(sprite.original_image, math.degrees(body.angle))

            # Recalculate the rect after rotation
            rotated_rect = rotated_image.get_rect(center=sprite.rect.center)

            # Update the sprite's image and rect
            sprite.image = rotated_image
            sprite.rect = rotated_rect

# To do: fix the slap dash screen height passing