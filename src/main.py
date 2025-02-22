import numpy as np
import open3d as o3d
import time

def main():
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name = "SIM", width = 1280, height = 720)
    dt = 0.001

    ## main loop ##
    while True:
        
        print("mainloop")
        time.sleep(dt)

if __name__ == "__main__":
    main()