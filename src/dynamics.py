import numpy as np
import sympy as sp

def dynamics(data_matrix: np.ndarray, dt: float, potential_str: str, ext_force: np.ndarray): #, collision_matrix: np.ndarray):
    m_vec = data_matrix[:,0] # Reading mass of every object

    x_vec = data_matrix[:,1] # Reading x center of mass potions of every object
    y_vec = data_matrix[:,2] # Reading y center of mass potions of every object
    z_vec = data_matrix[:,3] # Reading z center of mass potions of every object

    px_vec = data_matrix[:,4] # Reading center of mass momentum x component for every vector
    py_vec = data_matrix[:,5] # Reading center of mass momentum y component for every vector
    pz_vec = data_matrix[:,6] # Reading center of mass momentum z compenent for every vector

    ext_force_x = ext_force[:,0]
    ext_force_y = ext_force[:,1]
    ext_force_z = ext_force[:,2]

    n = len(x_vec) # Number of objects

    # Lizard Testing Potential Input
    for index in np.arange(0,len(potential_str)):
        if((potential_str[index] != "x") and (potential_str[index] != "y") 
           and (potential_str[index] != "z") and (potential_str[index] != "+")
            and (potential_str[index] != "-") and (potential_str[index] != "/")
            and (potential_str[index] != "*") and (potential_str[index] != " ")
            and (potential_str[index] != "np.exp") and (potential_str[index] != "np.sqrt")
            and (potential_str[index] != "1") and (potential_str[index] != "2")
            and (potential_str[index] != "3") and (potential_str[index] != "4")
            and (potential_str[index] != "5") and (potential_str[index] != "6")
            and (potential_str[index] != "7") and (potential_str[index] != "8")
            and (potential_str[index] != "9") and (potential_str[index] != "0")
            and (potential_str[index] != "m") and (potential_str[index] != "(")
            and (potential_str[index] != ")") and (potential_str[index] != ".")
           ):
            print(f"Potential function is invalid, can only depend on x,y,z and constants") 
            return(0)
    
    # x = sp.symbols(f'x:{n}')
    # y = sp.symbols(f'y:{n}')
    # z = sp.symbols(f'z:{n}')
    # px = sp.symbols(f'px:{n}')
    # py = sp.symbols(f'py:{n}')
    # pz = sp.symbols(f'pz:{n}')

    # Defining sybolic variables
    m = sp.symbols('m') 
    x = sp.symbols('x')
    y = sp.symbols('y')
    z = sp.symbols('z')
    px = sp.symbols('px')
    py = sp.symbols('py')
    pz = sp.symbols('pz')

    potential = sp.sympify(potential_str) # Defining the potential function symbolically
    dVdx = sp.diff(potential,x) # Differentiaing the potential with respect to x
    dVdy = sp.diff(potential,y) # Differentiaing the potential with respect to y
    dVdz = sp.diff(potential,z) # Differentiaing the potential with respect to z

    dVdx = sp.lambdify((x, y, z, m), dVdx, 'numpy')
    dVdy = sp.lambdify((x, y, z, m), dVdy, 'numpy')
    dVdz = sp.lambdify((x, y, z, m), dVdz, 'numpy')

    dx = np.zeros((n,1))
    dy = np.zeros((n,1))
    dz = np.zeros((n,1))
    dpx = np.zeros((n,1))
    dpy = np.zeros((n,1))
    dpz = np.zeros((n,1))

    dx = px_vec/m_vec * dt # x update
    dy = py_vec/m_vec * dt # y update
    dz = pz_vec/m_vec * dt # z update
    dpx = -dVdx(x_vec, y_vec, z_vec, m_vec)*dt + ext_force_x*dt # + collision_matrix @ np.ones((n,1)) * collision()
    dpy = -dVdy(x_vec, y_vec, z_vec, m_vec)*dt + ext_force_y*dt
    dpz = -dVdz(x_vec, y_vec, z_vec, m_vec)*dt + ext_force_z*dt

    x_vec = x_vec + dx # New x
    y_vec = y_vec + dy # New y
    z_vec = z_vec + dz # New z

    px_vec = px_vec + dpx # New px
    py_vec = py_vec + dpy # New py
    pz_vec = pz_vec + dpz # New pz

    data_matrix_output = np.column_stack((m_vec, x_vec, y_vec, z_vec, px_vec, py_vec, pz_vec)) # Constructing output matrix
    print(data_matrix_output)
    return(data_matrix_output)

dynamics(np.array([[1,2,3,5,3,2,1],
                  [4,5,6,5,3,2,1],
                  [7,8,9,5,3,2,1]]), 0.001, "x+y",np.zeros((3,3)))
