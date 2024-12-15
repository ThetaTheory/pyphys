from pyphys.objects import Body

def apply_gravity(body: Body, dt, gravity=9.8):
    body.velocity[1] -= gravity * body.mass * dt

# to do: figure out a way to infer gravity in friction from apply_gravity