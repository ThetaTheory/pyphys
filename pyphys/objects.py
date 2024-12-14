import math
from pyphys.vector import Vector2D

# Parent Body
# Child Rectangle, Circle, Polygon

class Body():
    def __init__(self, x, y, mass = 1, bounce = 0.5, name = None, is_static = False, static_friction=0.5, dynamic_friction=0.3):
        # name
        self.name = name
        # shape & position
        self.center = Vector2D(x, y)
        self.angle = 0    
        self.shape_type = None
        # velocity & force
        self.velocity = Vector2D(0, 0)
        self.angular_velocity = 0
        self.force = Vector2D(0, 0)
        # properties
        self.is_static = is_static
        self.mass = mass if not is_static else float("inf")
        self.inertia = None
        self.bounce = bounce
        self.static_friction = static_friction
        self.dynamic_friction = dynamic_friction
        # AABB min max
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
    
    def apply_force(self, fx, fy):
        if not self.is_static:
            acceleration = Vector2D(fx / self.mass, fy / self.mass)
            self.velocity += acceleration

    def update_aabb(self):
        if self.shape_type == "Polygon":
            # Get the transformed vertices
            object_vertices = self.get_vertices()  # In subclasses, get_vertices will account for rotation & translation
            # Update AABB by finding min/max of the object's vertices
            self.min_x = min(vertex.x for vertex in object_vertices)
            self.min_y = min(vertex.y for vertex in object_vertices)
            self.max_x = max(vertex.x for vertex in object_vertices)
            self.max_y = max(vertex.y for vertex in object_vertices)
        elif self.shape_type == "Circle":
            self.min_x = self.center.x - self.radius
            self.min_y = self.center.y - self.radius
            self.max_x = self.center.x + self.radius
            self.max_y = self.center.y + self.radius
        else:
            raise TypeError("Unknown shape")

    def get_vertices(self):
        """implemented in rectangle and polygon child classes"""
        raise NotImplementedError("This method must be implemented in child classes")

class Rectangle(Body):
    def __init__(self, x, y, width, height, mass = 1, bounce = 0.5, name = None, is_static = False, static_friction=0.5, dynamic_friction=0.3):
        super().__init__(x, y, mass, bounce, name, is_static, static_friction, dynamic_friction)
        self.width = width
        self.height = height
        self.shape_type = "Polygon"
        half_width = self.width / 2
        half_height = self.height / 2

        self.local_vertices = [
            Vector2D(-half_width, -half_height),
            Vector2D(half_width, -half_height),
            Vector2D(half_width, half_height),
            Vector2D(-half_width, half_height)
        ]
        #
        self.inertia = (1 / 12) * mass * (width**2 + height**2) if not is_static else float("inf")

        
    def get_axes(self):
        self.x_axis = Vector2D(math.cos(self.angle), math.sin(self.angle))
        self.y_axis = Vector2D(-math.sin(self.angle), math.cos(self.angle)) 
        
        return [self.x_axis, self.y_axis] 

    def get_vertices(self):
        return [vertex.rotate(self.angle).add(self.center) for vertex in self.local_vertices]

    def rotate(self, angle, in_radians=True):
        if not in_radians:
            angle = math.radians(angle)
        self.angle += angle

class Circle(Body):
    def __init__(self, x, y, radius, mass = 1, bounce = 0.5, name = None, is_static = False):
        super().__init__(x, y, mass, bounce, name, is_static)
        self.radius = radius
        self.shape_type = "Circle"
        self.inertia = (1 / 2) * mass * radius * radius if not is_static else float("inf")
        self.min_x = x - self.radius
        self.min_y = y - self.radius
        self.max_x = x + self.radius
        self.max_y = y + self.radius

    def rotate(self, angle, in_radians=True):
        if not in_radians:
            angle = math.radians(angle)
        self.angle += angle

class Polygon(Body):
    def __init__(self, x, y, vertices: list[Vector2D, list, tuple], mass=1, bounce=0.5, name=None, is_static=False):
        super().__init__(x, y, mass, bounce, name, is_static)
        centroid = (
            sum(vertex[0] for vertex in vertices) / len(vertices),
            sum(vertex[1] for vertex in vertices) / len(vertices),
        )

        self.local_vertices = [Vector2D(vertex[0] - centroid[0], vertex[1] - centroid[1]) for vertex in vertices]
      
        self.shape_type = "Polygon"
        self.inertia = self.calculate_inertia() if not is_static else float("inf")#
    
    
    def get_vertices(self):
        return [vertex.rotate(self.angle).add(self.center) for vertex in self.local_vertices]
    
    def calculate_inertia(self):
        # Initialize variables
        area = 0
        center = Vector2D(0, 0)
        mmoi = 0
        # Set the last vertex as the initial previous vertex
        prev = len(self.local_vertices) - 1
        # Iterate through each edge of the polygon
        for index in range(len(self.local_vertices)):
            a = self.local_vertices[prev]  # Previous vertex
            b = self.local_vertices[index]  # Current vertex
            # Calculate the area of the triangle formed by the edge
            area_step = a.cross(b) / 2
            # Calculate the centroid of the triangle
            center_step = (a + b) * (1 / 3)  
            # Calculate the moment of inertia for the triangle
            mmoi_step = (
            (a.cross(b) / 12)
            * (a.dot(a) + a.dot(b) + b.dot(b))  # MMOI formula for a triangle
            )
            # Update the centroid considering the new triangle
            center = (center * area + center_step * area_step) / (area + area_step) 
            # Accumulate the area and moment of inertia
            area += area_step
            mmoi += mmoi_step 
            # Move to the next edge
            prev = index 
        # Calculate the density of the polygon
        density = self.mass / area  
        # Adjust the moment of inertia with the density
        mmoi *= density 
        # Apply the parallel axis theorem to adjust for the centroid
        for vertex in self.local_vertices:
            mmoi += density * area * vertex.magnitude()**2
        # Return the final moment of inertia
        return mmoi


    def rotate(self, angle, in_radians=True):
        if not in_radians:
            angle = math.radians(angle)
        self.angle += angle

