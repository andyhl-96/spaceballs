from turtle import update
import numpy as np
import open3d as o3d
import time
from body import Body
from collision import check_collision
from dynamics import dynamics


origin = [0, 0, 0]
# holds all created objects
#objects = []
vis = o3d.visualization.Visualizer()
vis.create_window(window_name = "SIM", width = 1280, height = 720)
view_control = vis.get_view_control()
view_control.set_zoom(100)


# some helper functions
def create_sphere(position: list, radius: float, color: list, mass: float):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius, 20, False)
    sphere.translate(position)
    sphere.paint_uniform_color(color)
    body = Body(xyz = np.array(position), mass = mass, geom = radius, model = sphere)
    vis.add_geometry(Body.objects[-1].model)

def create_box(position: list, w: float, d: float, h: float, color: list, mass: float):
    box = o3d.geometry.TriangleMesh.create_box(width = w, depth = d, height = h)
    box.translate(position)
    box.paint_uniform_color(color)
    body = Body(xyz = np.array(position), mass = mass, model = box, geom = 0)
    vis.add_geometry(Body.objects[-1].model)

def update_viewport():
    for obj in Body.objects:
        vis.update_geometry(obj.model)

def update_dynamics(dynam_new):
    Body.dynamics_matrix = dynam_new
    for i in range(len(Body.dynamics_matrix)):
        bodypos = Body.objects[i].position()
        for j in range(1, 4):
            Body.dynamics_matrix[i, j] = bodypos[j - 1]

def main():
    global objects
    dt = 0.001
    global vis


    create_sphere([4, 2, 0], 0.5, [1, 0, 0], 1)
    create_sphere([4, 4, 0], 0.5, [0, 0, 1], 1)
    create_sphere([4, 6, 0], 0.5, [0, 1, 0], 1)
    create_box([0, 0, 0], 10, 10, 0.1, [0, 0, 0], 0)

    ## main loop ##
    while True:
        update_viewport()
        collisions = check_collision()
        # get updated mass, positions, momentums
        dynam = dynamics(Body.dynamics_matrix, dt, "0.5 * 1000 * x ** 2", np.zeros((len(Body.dynamics_matrix), 3)))

        vis.poll_events()
        vis.update_renderer()
        #if not 1 in collisions[0:]:
            #Body.objects[0].model.translate([0, -10 * dt, 0])
        vel = dynam[2, 4:7] / dynam[2, 0]
        Body.objects[2].model.translate((vel * dt).tolist())
        #if not 1 in collisions[1:]:
        vel = dynam[1, 4:7] / dynam[1, 0]
        Body.objects[1].model.translate((vel * dt).tolist())

        vel = dynam[0, 4:7] / dynam[0, 0]
        Body.objects[0].model.translate((vel * dt).tolist())

        update_dynamics(dynam)

        time.sleep(dt)

if __name__ == "__main__":
    main()