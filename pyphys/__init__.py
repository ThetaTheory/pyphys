# Import key components from submodules
from .engine import PhysicsEngine
from .objects import Body, Rectangle, Circle, Polygon
from .collision import collide
from .forces import apply_gravity
from .integration import Sprite, SpriteBinder