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
    body = Body(mass = mass, geom = radius, model = sphere)
    #objects.append(body)
    vis.add_geometry(Body.objects[-1].model)

def create_box(position: list, w: float, d: float, h: float, color: list, mass: float):
    box = o3d.geometry.TriangleMesh.create_box(width = w, depth = d, height = h)
    box.translate(position)
    box.paint_uniform_color(color)
    body = Body(mass = mass, model = box)
    #objects.append(body)
    vis.add_geometry(Body.objects[-1].model)

def update_viewport():
    for obj in Body.objects:
        vis.update_geometry(obj.model)

def main():
    global objects
    dt = 0.001
    global vis


    create_sphere([0, 4, 0], 1, [1, 0, 0], 0)
    create_sphere([0, 0, 0], 1, [0, 0, 1], 0)
    create_sphere([4, 0, 0], 1, [0, 1, 0], 0)
    print(Body.dynamics_matrix)
    #create_box([0, 0, 0], 10, 10, 0.1, [0, 0, 0], 0)

    ## main loop ##
    while True:
        update_viewport()
        collisions = check_collision()
        # get updated mass, positions, momentums
        dynam = dynamics(Body.dynamics_matrix, dt, "m*(-9.8)*y", np.zeros((len(Body.dynamics_matrix), 3)))
        #print(collisions)

        vis.poll_events()
        vis.update_renderer()
        if not 1 in collisions[0:]:
            #Body.objects[0].model.translate([0, -10 * dt, 0])
            vel = dynam[0, 4:7] / dynam[0, 0]
            Body.objects[0].model.translate(vel.tolist() * dt)
        if not 1 in collisions[2:]:
            #Body.objects[2].model.translate([-10 * dt, 0, 0])
            pass

        Body.dynamics_matrix = dynam
        time.sleep(dt)

if __name__ == "__main__":
    main()