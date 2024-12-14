from pyphys.objects import Body

def apply_gravity(body: Body, gravity=9.8):
    force_y = body.mass * gravity
    body.apply_force(0, force_y)

# def apply_friction(body: Body, friction=0.1):
#     friction_x = -friction * body.mass * 9.8 # fixed gravity for now
#     body.apply_force(friction_x if body.vx > 0 else -friction_x, 0)

# to do: figure out a way to infer gravity in friction from apply_gravity