from pyphys.objects import Body

def apply_gravity(body: Body, dt, gravity=9.8):
    gravity_force = body.mass * gravity
    body.apply_force(0, -gravity_force * dt)

# to do: figure out a way to infer gravity in friction from apply_gravity