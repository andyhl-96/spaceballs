from turtle import right
from body import Body
import numpy as np

class KDTNode:
    # takes an axis to partition space on, value to partition on, bodies in the partition
    def __init__(self, div_axis: int, val: float, bodies: list):
        self.div_axis = div_axis
        self.val = val
        self.bodies = bodies
        self.left = None
        self.right = None

def buildKDT(maxdepth: int = 0, height: int = 0, bodies: list = []):
    if height > maxdepth:
        return
    positions = []
    for body in bodies:
        positions.append(body.position())
    positions = np.array(positions)
    median = np.median(positions[:, 0])

    root = KDTNode(height % 3, median, bodies)

    # get subsets of bodies lt and gt median
    leftset = []
    rightset = []
    for body in bodies:
        if body.position()[height % 3] <= median:
            leftset.append(bodies)
        else:
            rightset.append(bodies)
    
    root.left = buildKDT((height + 1) % 3, leftset)
    root.right = buildKDT((height + 1) % 3, rightset)

    # probably get the leaves somehow

    return root


def distance(body1, body2):
    return np.sqrt(np.sum(np.power(body1.position() - body2.position(), 2)))

def check_collision():
    objects = Body.objects

    collision_matrix = np.zeros((len(objects), len(objects)))
    collision_vectors = np.zeros((len(objects), len(objects), 3))
    # perform a broad phase check then narrow phase
    for i in range(len(objects) - 1):
        for j in range(i + 1, len(objects)):
            if distance(objects[i], objects[j]) < (objects[i].geom + objects[j].geom):
                collision_matrix[i, j] = 1
                collision_matrix[j, i] = 1
                collision_vect = (objects[i].position() - objects[j].position()) / (distance(objects[i], objects[j]))
                collision_vectors[i, j] = collision_vect
                collision_vectors[j, i] = -1 * collision_vect


    return collision_matrix, collision_vectors