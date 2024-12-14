from pyphys.objects import Body
from pyphys.collision import aabb_collision, collide
from pyphys.forces import apply_gravity

# -- To do: Apply bounding volume hiearchy + raycast for more efficient collision detection

class PhysicsEngine:
    def __init__(self, bodies: list[Body], gravity = 9.8):
        self.bodies: list[Body] = bodies
        self._contact_points = []
        self.gravity = gravity

    def add(self, body: Body):
        self.bodies.append(body)

    def update_position(self, dt):
        for body in self.bodies:
            if body.is_static == False:
                body.center += body.velocity * dt
                body.angle += body.angular_velocity * dt
                body.update_aabb()
    
    def simulate_gravity(self, dt):
        for body in self.bodies:
            if not body.is_static:
                gravity_force = body.mass * self.gravity
                body.apply_force(0, -gravity_force * dt)  # Apply force downward

    # checks collision of all pairs in body list. Result:
    # 1. collision resolution handled by collide method
    # 2. all contact points of this instance saved until the next iteration
    def handle_collisions(self):
        self._contact_points = []
        for i in range(len(self.bodies) - 1):
            for j in range(i + 1, len(self.bodies)):
                # precaution; if same body, skip
                if self.bodies[i] == self.bodies[j]:
                    continue
                # if both static, skip
                if self.bodies[i].is_static and self.bodies[j].is_static:
                    continue

                # # Borad Phase Collision Detection -- AABB
                # if not aabb_collision(self.bodies[i], self.bodies[j]):
                #     continue
                # print(f'AABB collision detected between {i}: {self.bodies[i].name} and {j}: {self.bodies[j].name}')

                # Narrow Phase Collision Detection -- SAT
                contact_points = collide(self.bodies[i], self.bodies[j], True, False)
                if contact_points is None:
                    continue

                for point in contact_points:
                    if point is None:
                        continue
                    self._contact_points.append(point)

    def step(self, dt):
        self.simulate_gravity(dt)
        self.update_position(dt)
        self.handle_collisions()
