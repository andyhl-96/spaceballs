# this class is used to represent a physical body
# for which physics will be simulated
import numpy as np
import open3d as o3d

class Body:
    # static variable to keep track of all bodies
    # m, dx, dy, dz, dpx, pdy, dpz

    # model contains the position
    dynamics_matrix = np.zeros((1, 7))
    objects = []

    def __init__(self, 
                 mass: float = 0,
                 charge: float = 0,
                 moment: float = 0,
                 geom = None,
                 model = None):
        self.mass = mass
        self.charge = charge
        self.moment = moment
        # assumes to be sphere s radius
        self.geom = geom
        # open3d model
        self.model = model

        Body.objects.append(self)
    
    def position(self):
        if self.model == None:
            return np.array([0, 0, 0])
        position = np.array(self.model.vertices)
        center = np.mean(position, axis = 0)
        return center