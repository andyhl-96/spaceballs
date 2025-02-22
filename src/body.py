# this class is used to represent a physical body
# for which physics will be simulated
import numpy as np
import open3d as o3d

class Body:
    # static variable to keep track of all bodies
    dynamics_matrix = np.zeros((1, 6))
    def __init__(self, 
                 x_com: float = 0, 
                 y_com: float = 0, 
                 z_com: float = 0,
                 mass: float = 0,
                 charge: float = 0,
                 moment: float = 0,
                 geom = None,
                 model = None):
        self.x_com = x_com
        self.y_com = y_com
        self.z_com = z_com
        self.mass = mass
        self.charge = charge
        self.moment = moment
        self.geom = geom
        # open3d model
        self.model = model