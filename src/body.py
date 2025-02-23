# this class is used to represent a physical body
# for which physics will be simulated
import numpy as np
import open3d as o3d

class Body:
    # static variable to keep track of all bodies
    # m, dx, dy, dz, dpx, pdy, dpz

    # model contains the position
    dynamics_matrix = np.zeros((0, 8))
    objects = []

    # do not use xyz for position, its only initial position
    def __init__(self, 
                 xyz: np.ndarray,
                 mass: float = 1,
                 charge: float = 0,
                 moment: float = 0,
                 restitution: float = 1,
                 geom = None,
                 model = None,
                 exclude: bool = False):
        self.xyz = xyz
        self.mass = mass
        self.charge = charge
        self.moment = moment
        # assumes to be sphere s radius
        self.geom = geom
        # open3d model
        self.model = model
        self.exclude = exclude

        Body.objects.append(self)
        dynamics = np.array([mass, 
                             charge,
                             xyz[0], 
                             xyz[1],
                             xyz[2],
                             np.random.normal(0, 100),
                             np.random.normal(0, 100),
                             np.random.normal(0, 100)])
        if len(Body.dynamics_matrix) == 0:
            Body.dynamics_matrix = np.array([dynamics])
        else:
            Body.dynamics_matrix = np.vstack([Body.dynamics_matrix, dynamics])

    
    def position(self):
        if self.model == None:
            return np.array([0, 0, 0])
        position = np.array(self.model.vertices)
        center = np.mean(position, axis = 0)
        return center