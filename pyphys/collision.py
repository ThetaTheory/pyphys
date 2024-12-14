from pyphys.objects import Body, Circle, Polygon
from pyphys.vector import Vector2D


### Main Collision Handler ###

def collide(body_1: Body, body_2: Body, include_rotation = True, include_friction = True):
    # detect collision, find normal and depth
    if body_1.shape_type == "Polygon" and body_2.shape_type == "Polygon":
        normal, depth = polygons_sat_collision(body_1, body_2) 
    elif body_1.shape_type == "Circle" and body_2.shape_type == "Circle":
        normal, depth = circles_collision(body_1, body_2)
    elif body_1.shape_type == "Polygon" and body_2.shape_type == "Circle":
        normal, depth = polygon_circle_collision(body_1, body_2)
    elif body_1.shape_type == "Circle" and body_2.shape_type == "Polygon":
        normal, depth = polygon_circle_collision(body_2, body_1)

    if normal is None or depth is None:
        return # exit if no collision detected
    
    # print(f'collision detected between {body_1.name} and {body_2.name}')
    
    # find contact points if collision detected
    if body_1.shape_type == "Polygon" and body_2.shape_type == "Polygon":
        contact_points = polygons_contact_points(body_1, body_2)
    elif body_1.shape_type == "Circle" and body_2.shape_type == "Circle":
        contact_points = circles_contact_points(body_1, body_2)
    elif body_1.shape_type == "Polygon" and body_2.shape_type == "Circle":
        contact_points = polygon_circle_contact_points(body_1, body_2)
    elif body_1.shape_type == "Circle" and body_2.shape_type == "Polygon":
        contact_points = polygon_circle_contact_points(body_2, body_1)
        normal = -normal

    # print(f'contact points found between {body_1.name} and {body_2.name}')
        
    # collision response
    if include_rotation:
        response_with_rotation(body_1, body_2, normal, depth, contact_points)
        # print('responded with rotation')
    # elif include_friction:
    #     response_with_friction(body_1, body_2, normal, depth, contact_points)
    else:
        response(body_1, body_2, normal, depth)
        # print('responded without rotation')

    return contact_points


### Collision Detection ###

## AABB
def aabb_collision(body1, body2):
    return not (
        body1.max_x < body2.min_x or  # body1 is to the left of body2
        body1.min_x > body2.max_x or  # body1 is to the right of body2
        body1.max_y < body2.min_y or  # body1 is below body2
        body1.min_y > body2.max_y     # body1 is above body2
    )

## Polygon-Polygon SAT
def polygons_sat_collision(polygon_1: Polygon, polygon_2: Polygon):
    normal = Vector2D(0, 0)
    depth = float('inf')
    
    vertices1 = polygon_1.get_vertices()
    vertices2 = polygon_2.get_vertices()
    
    # check projection for polygon_1 normals, find minimum depth
    normal, depth = check_projection(vertices1, vertices2, normal, depth)
    if depth is None:  # No overlap found
        return None, None
    # check projection for polygon_2 normals, find minimum depth\
    normal, depth = check_projection(vertices2, vertices1, normal, depth)
    if depth is None:  # No overlap found
        return None, None
    # now that collision point's normal has been found, adjust so it points from polygon_2 to polygon_1
    direction = (polygon_1.center - polygon_2.center).normalize()
    if direction.dot(normal) < 0:
        normal *= -1

    return normal, depth

## Polygon-Circle SAT
def polygon_circle_collision(polygon: Polygon, circle: Circle):
    assert polygon.shape_type == "Polygon" and circle.shape_type == "Circle", \
        "Shape types of polygon and circle must be 'Polygon' and 'Circle' respectively."
    
    normal = Vector2D(0, 0)
    depth = float('inf')   
    vertices = polygon.get_vertices()

    # check projection for polygon normals, find minimum depth
    for i in range(len(vertices)):
        va = vertices[i]
        vb = vertices[(i + 1) % len(vertices)]
        edge = vb - va
        axis = Vector2D(-edge.y, edge.x).normalize()
        # project polygon onto axis
        min_a, max_a = project_vertices(vertices, axis)
        # project circle onto axis
        min_b, max_b = project_circle(circle.center, circle.radius, axis)

        if max_a <= min_b or max_b <= min_a:
            return None, None
        
        axis_depth = min(max_b - min_a, max_a - min_b)
        if axis_depth < depth:
            depth = axis_depth
            normal = axis

    # check for case where circle collides with vertex, not edge
    cp = find_closest_polygon_point(circle.center, vertices)
    axis = (cp - circle.center).normalize()

    min_a, max_a = project_circle(circle.center, circle.radius, axis)
    min_b, max_b = project_vertices(vertices, axis)

    if max_a <= min_b or max_b <= min_a:
        return None, None
    
    axis_depth = min(max_b - min_a, max_a - min_b)
    if axis_depth < depth:
        depth = axis_depth
        normal = axis

    # adjust normal direction
    direction = (polygon.center - circle.center).normalize()
    if direction.dot(normal) < 0:
        normal *= -1
        
    return normal, depth

## Circle-Circle
def circles_collision(body_1: Circle, body_2: Circle):
    assert body_1.shape_type == "Circle" and body_2.shape_type == "Circle", \
        "Both body_1 and body_2 must be of shape_type 'Circle' for Circle collision."
    
    # if distance between two circle centers is smaller than sum of two radius, it's colliding
    distance = Vector2D.distance(body_1.center, body_2.center)
    if distance >= body_1.radius + body_2.radius:
        return None, None
    
    normal = (body_1.center - body_2.center).normalize()
    depth  = body_1.radius + body_2.radius - distance

    return normal, depth


## Helper Functions for SAT ##

## project vertices onto an axis to find the two ends of the projection
def project_vertices(vertices: list[Vector2D], axis: Vector2D):
    # initialise extreme ends
    min_proj = float('inf')
    max_proj = float('-inf')
    # project all vertexes onto axis and sort for min and max projections
    for v in vertices:
        proj = v.dot(axis) # project vertex onto axis
        if proj < min_proj:
            min_proj = proj # update min projection
        if proj > max_proj:
            max_proj = proj # update max projection
    return min_proj, max_proj

## find collision point
def check_projection(vertices1, vertices2, normal, depth):
    # project all edges of two polygons onto all normals of the first polygon
    for i in range(len(vertices1)):
        # find normal axis
        va = vertices1[i]
        vb = vertices1[(i + 1) % len(vertices1)]
        edge = vb - va
        axis = Vector2D(-edge.y, edge.x).normalize()
        # find projections on the axis for both polygons
        min_a, max_a = project_vertices(vertices1, axis)
        min_b, max_b = project_vertices(vertices2, axis)
        # exit iteration if no overlap
        if min_a >= max_b or min_b >= max_a:
            return None, None
        # find depth of overlap
        axis_depth = min(max_b - min_a, max_a - min_b)
        # find smallest depth, i.e. the collision point
        if axis_depth < depth:
            depth = axis_depth
            normal = axis
    return normal, depth

## project circle onto an axis
def project_circle(center, radius: float, axis: Vector2D):
    direction = axis.normalize()
    direction_and_radius = direction * radius

    p1 = center + direction_and_radius
    p2 = center - direction_and_radius

    min_proj = p1.dot(axis)
    max_proj = p2.dot(axis)

    if min_proj > max_proj:
        min_proj, max_proj = max_proj, min_proj

    return min_proj, max_proj

## find closet polygon vertex from circle
def find_closest_polygon_point(circle_center: Vector2D, vertices: list[Vector2D]):
    index = -1
    min_distance = float('inf')

    for i, v in enumerate(vertices):
        dist = Vector2D.distance(v, circle_center)
        if dist < min_distance:
            min_distance = dist
            index = i

    return vertices[index]


### Find Contact Point ###

def polygons_contact_points(polygon_1: Polygon, polygon_2: Polygon):
    epsilon = 0.0005
    min_distance = float('inf')
    # 1~2 possible contact points because polygon is convex (single point or line segment)
    contact_point_1 = None
    contact_point_2 = None

    # check if vertex of polygon_1 is in contact with edge of polygon_2
    for i in range(len(polygon_1.get_vertices())): # for each vertex in polygon_1
        vp = polygon_1.get_vertices()[i]
        for j in range(len(polygon_2.get_vertices())): # for each edge in polygon_2
            # find closest point to polygon_1 vertex
            va = polygon_2.get_vertices()[j]
            vb = polygon_2.get_vertices()[(j + 1) % len(polygon_2.get_vertices())]
            cp, distance = point_to_line_segment_projection(vp, va, vb)

            # if contact_point_1 exists and new contact point is along the same line of contact and not too far from contact_point_1, assign contact_point_2
            if contact_point_1 is not None and abs(distance - min_distance) < epsilon and not cp.distance_to(contact_point_1) < epsilon:
                contact_point_2 = cp
            # else, update contact_point_1 with new closest contact point
            elif distance < min_distance:
                min_distance = distance
                contact_point_2 = None
                contact_point_1 = cp

    # check if vertex of polygon_2 is in contact with edge of polygon_1
    for i in range(len(polygon_2.get_vertices())):
        vp = polygon_2.get_vertices()[i]
        for j in range(len(polygon_1.get_vertices())):
            va = polygon_1.get_vertices()[j]
            vb = polygon_1.get_vertices()[(j + 1) % len(polygon_1.get_vertices())]
            cp, distance = point_to_line_segment_projection(vp, va, vb)

            if contact_point_1 is not None and abs(distance - min_distance) < epsilon and not cp.distance_to(contact_point_1) < epsilon:
                contact_point_2 = cp
            elif distance < min_distance:
                min_distance = distance
                contact_point_2 = None
                contact_point_1 = cp

    return [cp for cp in [contact_point_1, contact_point_2] if cp is not None]

def polygon_circle_contact_points(polygon: Polygon, circle: Circle):
    min_distance = float('inf')
    vertices = polygon.get_vertices()

    # on each edge in polygon, find closest point to circle center
    for i in range(len(vertices)):
        va = vertices[i]
        vb = vertices[(i + 1) % len(vertices)]
        cp, distance = point_to_line_segment_projection(circle.center, va, vb)

        if distance < min_distance:
            min_distance = distance
            contact_point = cp
        
    return [contact_point]

def circles_contact_points(body_1: Circle, body_2: Circle):
    normal = (body_2.center - body_1.center).normalize()
    contact_point = body_1.center + normal * body_1.radius
    return [contact_point]

## Helper Functions for Contact Point ##

## finds closest point to point on a ab line segment
def point_to_line_segment_projection(point: Vector2D, a: Vector2D, b: Vector2D):
    ab = b - a 
    ap = point - a 
    proj = ap.dot(ab)
    # find projection of point on ab line segment
    d = proj / ab.dot(ab) 
    if d <= 0:
        contact_point = a
    elif d >= 1:
        contact_point = b
    else: 
        contact_point = a + ab * d
    
    distance = Vector2D.distance(contact_point, point)

    return contact_point, distance # return projection contact point and distance between point and contact point


### Collision Response ###

## Rotationless collision response
def response(body_1: Body, body_2: Body, normal_vector: Vector2D, penetration_depth: float):
    # Reverse the normal vector to apply impulse
    normal_vector *= -1
    # Separate the bodies to prevent overlap
    separate_bodies(body_1, body_2, normal_vector, penetration_depth)

    relative_velocity = body_2.velocity - body_1.velocity   # how fast they were moving toward/away from each other
    penetration_velocity = relative_velocity.dot(normal_vector) # that along normal axis
    # Skip if bodies are moving away from each other
    if penetration_velocity > 0:
        return
    # Compute the coefficient of restitution (bounciness) (arbitrary simplification)
    r = min(body_1.bounce, body_2.bounce)  # range: 0~1
    # Compute the impulse scalar (j)
    j = -(1 + r) * penetration_velocity
    j /= (1 / body_1.mass if not body_1.is_static else 0) + (1 / body_2.mass if not body_2.is_static else 0)
    # Compute the impulse vector
    impulse = normal_vector * j
    # Update velocities for body_1 and body_2 if they are not static
    if not body_1.is_static:
        body_1.velocity -= impulse / body_1.mass
    if not body_2.is_static:
        body_2.velocity += impulse / body_2.mass

## Rotation collision response
def response_with_rotation(body_1: Body, body_2: Body, normal_vector: Vector2D, penetration_depth: float, contact_points: list[Vector2D]):
    # Reverse the normal vector to apply impulse
    normal_vector *= -1  
    # Separate the bodies to prevent overlap
    separate_bodies(body_1, body_2, normal_vector, penetration_depth)
    # contact point
    if len(contact_points) == 2: 
        contact_point = (contact_points[0] + contact_points[1]) / 2
    else:
        contact_point = contact_points[0]
    # vectors from the body centers to the contact point
    r_1 = contact_point - body_1.center  
    r_2 = contact_point - body_2.center 
    # perpendicular vectors for angular velocity calculation
    r_1_perp = Vector2D(-r_1.y, r_1.x) 
    r_2_perp = Vector2D(-r_2.y, r_2.x)  
    # relative velocity at the contact point
    relative_velocity = (body_2.velocity + r_2_perp * body_2.angular_velocity) - (body_1.velocity + r_1_perp * body_1.angular_velocity)  
    penetration_velocity = relative_velocity.dot(normal_vector)  
    # Skip if bodies are moving away
    if penetration_velocity > 0:
        return
    # Compute the coefficient of restitution (bounciness) (arbitrary simplification)
    r = min(body_1.bounce, body_2.bounce) 
    # Compute the impulse scalar (j) 
    numerator = -(1 + r) * penetration_velocity
    denom = ((1 / body_1.mass if not body_1.is_static else 0) + 
             (1 / body_2.mass if not body_2.is_static else 0) +
             (r_1_perp.dot(r_1_perp) / body_1.inertia if not body_1.is_static else 0) +
             (r_2_perp.dot(r_2_perp) / body_2.inertia if not body_2.is_static else 0)) # account for mass and rotational inertia
    j = numerator / denom 
    # Compute the impulse vector
    impulse = normal_vector * j  
    # Update velocities for body_1 and body_2 if they are not static
    if not body_1.is_static:
        body_1.velocity -= impulse / body_1.mass
        body_1.angular_velocity -= r_1_perp.dot(impulse) / body_1.inertia
    if not body_2.is_static:
        body_2.velocity += impulse / body_2.mass
        body_2.angular_velocity += r_2_perp.dot(impulse) / body_2.inertia



## Helper Functions for Collision Response ##

def separate_bodies(body_1: Body, body_2: Body, normal, penetration_depth):
    separation_vector = normal * penetration_depth
    if body_1.is_static:
        body_2.center += separation_vector
    elif body_2.is_static:
        body_1.center -= separation_vector
    else:
        body_1.center -= separation_vector / 2
        body_2.center += separation_vector / 2