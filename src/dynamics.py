import numpy as np
import sympy as sp
from collision import check_collision

def dynamics(data_matrix: np.ndarray, dt: float, potential_str: str, ext_force: np.ndarray, e: float):
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

    collision_matrix, collision_vectors = check_collision()

    px_vec = px_vec + collision_update(collision_matrix, px_vec, py_vec, pz_vec,collision_vectors,m_vec,e)[:,0]
    py_vec = py_vec + collision_update(collision_matrix, px_vec, py_vec, pz_vec,collision_vectors,m_vec,e)[:,1]
    pz_vec = pz_vec + collision_update(collision_matrix, px_vec, py_vec, pz_vec,collision_vectors,m_vec,e)[:,2]

    dx = px_vec/m_vec * dt # x update
    dy = py_vec/m_vec * dt # y update
    dz = pz_vec/m_vec * dt # z update
    dpx = -dVdx(x_vec, y_vec, z_vec, m_vec)*dt + ext_force_x * dt
    dpy = -dVdy(x_vec, y_vec, z_vec, m_vec)*dt + ext_force_y * dt
    dpz = -dVdz(x_vec, y_vec, z_vec, m_vec)*dt + ext_force_z * dt 

    x_vec = x_vec + dx # New x
    y_vec = y_vec + dy # New y
    z_vec = z_vec + dz # New z

    px_vec = px_vec + dpx # New px
    py_vec = py_vec + dpy # New py
    pz_vec = pz_vec + dpz # New pz

    data_matrix_output = np.column_stack((m_vec, dx, dy, dz, px_vec, py_vec, pz_vec)) # Constructing output matrix

    # print(data_matrix_output)
    return(data_matrix_output)

def collision_update(collision_matrix: int,p0x: np.ndarray,p0y: np.ndarray,p0z: np.ndarray,collision_vectors: np.ndarray, m_vec: np.ndarray, e: float):
    
    p0_matrix = np.column_stack((p0x,p0y,p0z)) # Creating a matrix of momentum vectors
    num_collisions = collision_matrix @ np.ones(len(collision_matrix[:,0])) # Vector of number of collisions for each ball
    
    dp_vec = np.zeros((len(num_collisions), 3))

    for index in range(len(num_collisions)):
        match num_collisions[index]: 
            # colliding with 1 other body
            case 1: 
                collide_partner_ind = np.nonzero(collision_matrix[index,:]) # Index of ball that is being collided with
                p0a = p0_matrix[index,:] # Momentum of ball of interest
                p0b = p0_matrix[collide_partner_ind,:] # Momentum of colliding ball
                perp_unit_vec = collision_vectors[index, collide_partner_ind] # Pulling unit vector pointing to collision location for ball of interest 
                perp_unit_vec = perp_unit_vec[0, 0]
                p0a_perp = np.dot(p0a, perp_unit_vec) * perp_unit_vec# Projecting momentum in direction of collision for ball of interest
                p0b_perp = np.dot(p0b, perp_unit_vec) * perp_unit_vec# Projecting momentum in direction of collision for ball that is being collided with
                
                # Trying to create a vector perpendicular to perp_unit_vec
                
                while_index = 0
                error = 1
                while error > 0.0000001:
                    if perp_unit_vec[0] != 0:
                        y_try_par = np.random.random()
                        z_try_par = np.random.random()
                        x_try_par = -(perp_unit_vec[1]*y_try_par + perp_unit_vec[2]*z_try_par) / perp_unit_vec[0]
                    elif perp_unit_vec[1] != 0:
                        x_try_par = np.random.random()
                        z_try_par = np.random.random()
                        y_try_par = -(perp_unit_vec[0]*x_try_par + perp_unit_vec[2]*z_try_par) / perp_unit_vec[1]
                    elif perp_unit_vec[2] != 0:
                        x_try_par = np.random.random()
                        y_try_par = np.random.random()
                        z_try_par = -(perp_unit_vec[0]*x_try_par + perp_unit_vec[1]*y_try_par) / perp_unit_vec[2]
                    par_unit_vec = (np.array([x_try_par, y_try_par, z_try_par])) / np.linalg.norm((np.array([x_try_par,y_try_par,z_try_par]))) # Vector Paralell to collision plane
                    error = abs(np.dot(par_unit_vec, perp_unit_vec))
                    while_index = while_index + 1
                par_unit_vec2 = np.cross(par_unit_vec,perp_unit_vec) / np.linalg.norm(np.cross(par_unit_vec,perp_unit_vec)) # 2nd Vector Paralell to collision plane

                p0a_par1 = np.dot(par_unit_vec,p0a) * par_unit_vec # p0a projection into paralell dir 1 (conserved momentum)
                p0a_par2 = np.dot(par_unit_vec2,p0a) * par_unit_vec2 # p0a projection into paralell dir 2 (conserved momentum)

                #p0b_par1 = np.dot(par_unit_vec,p0b) * par_unit_vec # p0b projection into paralell dir 1 (conserved momentum)
                #p0b_par2 = np.dot(par_unit_vec2,p0b) * par_unit_vec2 # p0b projection into paralell dir 2 (conserved momentum)

                pfa_mag = (-e*m_vec[collide_partner_ind] * np.linalg.norm(p0a_perp) + m_vec[index] * (np.linalg.norm(p0a_perp)+np.linalg.norm(p0b_perp) + e*np.linalg.norm(p0b_perp))) / (m_vec[index] + m_vec[collide_partner_ind])
                pfa = pfa_mag * perp_unit_vec + p0a_par1 + p0a_par2
                dp_vec[index] = pfa - p0a
                dp_vec[collide_partner_ind] = p0a - pfa
                break
            case _:
                dp_vec[index] = np.array([0,0,0])
    # debug
    return(dp_vec)


# dynamics(np.array([[1,2,3,5,3,2,1],
#                   [4,5,6,5,3,2,1],
#                   [7,8,9,5,3,2,1]]), 0.001, "x+y",np.zeros((3,3)))
