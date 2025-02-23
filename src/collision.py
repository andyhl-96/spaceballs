from turtle import right
from body import Body
import numpy as np


# use this later for a broad phase check
class KDTNode:
    # takes an axis to partition space on, value to partition on, bodies in the partition
    def __init__(self, div_axis: int, val: float, bodies: list):
        self.div_axis = div_axis
        self.val = val
        self.bodies = bodies
        self.left = None
        self.right = None

def buildKDT(maxdepth: int = 0, height: int = 0, bodies: list = []):
    if height > maxdepth or len(bodies) == 0:
        return None
    
    # get the median position
    positions = []
    for body in bodies:
        positions.append(body.position())
    positions = np.array(positions)
    median = np.median(positions[:, 0])

    root = KDTNode(height % 3, height, bodies)

    # get subsets of bodies lt and gt median
    leftset = []
    rightset = []
    for body in bodies:
        # height determines the axis to divide on
        if body.position()[height % 3] + body.geom <= median:
            leftset.append(body)
        else:
            rightset.append(body)
    
    root.left = buildKDT(maxdepth, height + 1, leftset)
    root.right = buildKDT(maxdepth, height + 1, rightset)

    # probably get the leaves somehow

    return root

def print_leaves(root: KDTNode):
    if root.left == None and root.right == None:
        print(f"box: {root.bodies}")
        return
    if root.left != None:
        print_leaves(root.left)
    if root.right != None:
        print_leaves(root.right)
    

def distance(body1, body2):
    return np.sqrt(np.sum(np.power(body1.position() - body2.position(), 2)))

# simple collision checker between balls and walls
# bounds for now will bound sphere centers to an
# axis aligned box (xlb, xub, ylb, yub, zlb, zub)
def check_collision(bounds: tuple):
    objects = Body.objects
    # root = buildKDT(12, 0, objects)
    # print("TREE:")
    # print(f"ROOT: {root.bodies}")
    # print_leaves(root)


    # matrix to see what is colliding with what
    collision_matrix = np.zeros((len(objects), len(objects)))
    # vectors normal to collisions
    collision_vectors = np.zeros((len(objects), len(objects), 3))
    # holds collision vector between balls and walls
    collision_wall = np.zeros((len(objects), 3))
    # perform a broad phase check then narrow phase TODO
    # simple stupid check between all objects
    for i in range(len(objects) - 1):
        for j in range(i + 1, len(objects)):
            if distance(objects[i], objects[j]) < (objects[i].geom + objects[j].geom):
                collision_matrix[i, j] = 1
                collision_matrix[j, i] = 1
                collision_vect = (objects[i].position() - objects[j].position()) / (distance(objects[i], objects[j]))
                collision_vectors[i, j] = collision_vect
                collision_vectors[j, i] = -1 * collision_vect

        # check if object i is colliding with a bound
        pos = objects[i].position()
        if pos[0] < bounds[0]:
            collision_wall[i] += np.array([1, 0, 0])
        elif pos[0] > bounds[1]:
            collision_wall[i] += np.array([-1, 0, 0])
        if pos[1] < bounds[2]:
            collision_wall[i] += np.array([0, 1, 0])
        elif pos[1] > bounds[3]:
            collision_wall[i] += np.array([0, -1, 0])
        if pos[2] < bounds[4]:
            collision_wall[i] += np.array([0, 0, 1])
        elif pos[2] > bounds[5]:
            collision_wall[i] += np.array([0, 0, -1])
    # check the last ball
    pos = objects[-1].position()
    if pos[0] < bounds[0]:
        collision_wall[-1] += np.array([1, 0, 0])
    elif pos[0] > bounds[1]:
        collision_wall[-1] += np.array([-1, 0, 0])
    if pos[1] < bounds[2]:
        collision_wall[-1] += np.array([0, 1, 0])
    elif pos[1] > bounds[3]:
        collision_wall[-1] += np.array([0, -1, 0])
    if pos[2] < bounds[4]:
        collision_wall[-1] += np.array([0, 0, 1])
    elif pos[2] > bounds[5]:
        collision_wall[-1] += np.array([0, 0, -1])


    return collision_matrix, collision_vectors, collision_wall