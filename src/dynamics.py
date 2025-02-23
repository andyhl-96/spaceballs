import numpy as np
import sympy as sp
from collision import check_collision

# solve the potential function once because solving it every ms
# is silly
def solve_potential(potential_str: str, data_matrix: np.ndarray):
    mass_vec = data_matrix[:, 0]
    x_vec = data_matrix[:, 1]
    y_vec = data_matrix[:, 2]
    z_vec = data_matrix[:, 3]
    # Defining sybolic variables
    m = sp.symbols('m')
    q = sp.symbols('q')
    x = sp.symbols('x')
    y = sp.symbols('y')
    z = sp.symbols('z')
    px = sp.symbols('px')
    py = sp.symbols('py')
    pz = sp.symbols('pz')

    potential = sp.sympify(potential_str) # Defining the potential function symbolically
    dVdx = sp.diff(potential, x) # Differentiaing the potential with respect to x
    dVdy = sp.diff(potential, y) # Differentiaing the potential with respect to y
    dVdz = sp.diff(potential, z) # Differentiaing the potential with respect to z

    dVdx = sp.lambdify((x, y, z, m, q), dVdx, 'numpy')
    dVdy = sp.lambdify((x, y, z, m, q), dVdy, 'numpy')
    dVdz = sp.lambdify((x, y, z, m, q), dVdz, 'numpy')

    return (dVdx, dVdy, dVdz)

    

def dynamics(data_matrix: np.ndarray, dt: float, gradV: tuple, ext_force: np.ndarray, e: float, bounds: tuple, sim_grav: bool):
    m_vec = data_matrix[:, 0] # Reading mass of every object
    q_vec = data_matrix[:, 7] # get charges

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

    dx = np.zeros((n,1))
    dy = np.zeros((n,1))
    dz = np.zeros((n,1))
    dpx = np.zeros((n,1))
    dpy = np.zeros((n,1))
    dpz = np.zeros((n,1))

    collision_matrix, collision_vectors, collision_walls = check_collision(bounds)

    # get momentum update from collision
    dp, matrix = collision_update(collision_matrix, collision_walls, px_vec, py_vec, pz_vec, collision_vectors, m_vec, e)
    grav_x, grav_y, grav_z = gravitation(x_vec, y_vec, z_vec, m_vec)
    elec_x, elec_y, elec_z = electrostatic(x_vec, y_vec, z_vec, q_vec)
    mag_x, mag_y, mag_z = magnetism(x_vec, y_vec, z_vec, q_vec, px_vec, py_vec, pz_vec, m_vec)

    if not sim_grav:
        grav_x = 0
        grav_y = 0
        grav_z = 0
    
    # constants
    G = 6900
    ke = 690
    r = 0.0001
    km = r * ke

    px_vec = matrix @ px_vec + dp[:,0]
    py_vec = matrix @ py_vec + dp[:,1]
    pz_vec = matrix @ pz_vec + dp[:,2]

    # calculate change in momentum
    dx = px_vec / m_vec * dt # x update
    dy = py_vec / m_vec * dt # y update
    dz = pz_vec / m_vec * dt # z update
    dpx = -gradV[0](x_vec, y_vec, z_vec, m_vec, q_vec) * dt + ext_force_x * dt + G * grav_x * dt - ke * elec_x * dt + km * mag_x * dt
    dpy = -gradV[1](x_vec, y_vec, z_vec, m_vec, q_vec) * dt + ext_force_y * dt + G * grav_y * dt - ke * elec_y * dt + km * mag_y * dt
    dpz = -gradV[2](x_vec, y_vec, z_vec, m_vec, q_vec) * dt + ext_force_z * dt + G * grav_z * dt - ke * elec_z * dt + km * mag_z * dt

    # this somehow fixes gravitation. do not ask how
    dpx = dpx[:, 0]
    dpy = dpy[:, 0]
    dpz = dpz[:, 0]
    
    x_vec = x_vec + dx # New x
    y_vec = y_vec + dy # New y
    z_vec = z_vec + dz # New z

    px_vec = px_vec + dpx # New px
    py_vec = py_vec + dpy # New py
    pz_vec = pz_vec + dpz # New pz

    # used to get velocity of each body
    data_matrix_output = np.column_stack((m_vec, dx, dy, dz, px_vec, py_vec, pz_vec, q_vec)) # Constructing output matrix

    return(data_matrix_output)

def collision_update(collision_matrix: int, collision_walls: np.ndarray, p0x: np.ndarray,p0y: np.ndarray,p0z: np.ndarray,collision_vectors: np.ndarray, m_vec: np.ndarray, e: float):
    epsilon = 0.001 # See where this is used (if difference in momenta is below this the momenta are forced to be equal)

    p0_matrix = np.column_stack((p0x, p0y, p0z)) # Creating a matrix of momentum vectors
    num_collisions = collision_matrix @ np.ones(len(collision_matrix[:,0])) # Vector of number of collisions for each ball
    
    dp_vec = np.zeros((len(num_collisions), 3))
    wall_dp_vec = np.zeros((len(num_collisions), 3))


    # go through all balls
    for index in range(len(num_collisions)):
        # momentum of current ball
        p0a = p0_matrix[index,:]

        # only check collisions if ball to wall vector is non 0
        if np.linalg.norm(collision_walls[index,:]) != 0:

            # negative normal between ball and wall
            perp_unit_vec = -collision_walls[index, :] / np.linalg.norm(collision_walls[index, :])
            # vectors parallel to wall for plane
            # momentum conserved in these dirs
            par_unit_vec, par_unit_vec2 = find_paralell_unit_vecs(perp_unit_vec, 0.0000001)
            # momentum in first parallel direction
            p0a_par1 = np.dot(par_unit_vec,p0a) * par_unit_vec # p0a projection into paralell dir 1 (conserved momentum)
            # momentum in second parallel direction, both consrrvs
            p0a_par2 = np.dot(par_unit_vec2,p0a) * par_unit_vec2 # p0a projection into paralell dir 2 (conserved momentum)
            # projection of momentum in perpendicular direction
            # not conserved
            p0a_perp = np.dot(p0a,perp_unit_vec) * perp_unit_vec
            # can maybe extend here for individual resitutions
            pfa_mag = -e * np.linalg.norm(p0a_perp)
            # final momentum vector, parallel + perpendicular vectors
            pfa = pfa_mag * perp_unit_vec + p0a_par1 + p0a_par2
            wall_dp_vec[index] = pfa - p0a
            matrix = np.eye(len(num_collisions))

        # collisions for balls to balls
        # case of 1 ball is known to be correct. general case may not be but is good enough
        if num_collisions[index] == 1:
            collide_partner_ind = np.nonzero(collision_matrix[index,:]) # Index of ball that is being collided with
            p0a = p0_matrix[index,:] # Momentum of ball of interest
            p0b = p0_matrix[collide_partner_ind,:] # Momentum of colliding ball
            perp_unit_vec = collision_vectors[index, collide_partner_ind] # Pulling unit vector pointing to collision location for ball of interest 
            perp_unit_vec = perp_unit_vec[0, 0]
            p0a_perp = np.dot(p0a, perp_unit_vec) * perp_unit_vec# Projecting momentum in direction of collision for ball of interest
            p0b_perp = np.dot(p0b, perp_unit_vec) * perp_unit_vec# Projecting momentum in direction of collision for ball that is being collided with
            
            # Trying to create a vector perpendicular to perp_unit_vec
            
            par_unit_vec, par_unit_vec2 = find_paralell_unit_vecs(perp_unit_vec,0.0000001)

            p0a_par1 = np.dot(par_unit_vec,p0a) * par_unit_vec # p0a projection into paralell dir 1 (conserved momentum)
            p0a_par2 = np.dot(par_unit_vec2,p0a) * par_unit_vec2 # p0a projection into paralell dir 2 (conserved momentum)

            #p0b_par1 = np.dot(par_unit_vec,p0b) * par_unit_vec # p0b projection into paralell dir 1 (conserved momentum)
            #p0b_par2 = np.dot(par_unit_vec2,p0b) * par_unit_vec2 # p0b projection into paralell dir 2 (conserved momentum)

            # can extend here for individual resitutions
            pfa_mag = (-e*m_vec[collide_partner_ind] * np.linalg.norm(p0a_perp) + m_vec[index] * (np.linalg.norm(p0a_perp)+np.linalg.norm(p0b_perp) + e*np.linalg.norm(p0b_perp))) / (m_vec[index] + m_vec[collide_partner_ind])
            pfa = pfa_mag * perp_unit_vec + p0a_par1 + p0a_par2
            if abs(np.dot(p0a,perp_unit_vec) - np.dot(p0b,perp_unit_vec)) > epsilon:
                dp_vec[index] = pfa - p0a
                dp_vec[collide_partner_ind] = p0a - pfa
                matrix = np.eye(len(num_collisions))
                
            else:
                dp_vec[index] = np.array([0,0,0])
                dp_vec[collide_partner_ind] = np.array([0,0,0])
                matrix = np.zeros((len(num_collisions),len(num_collisions)))
                matrix[index,collide_partner_ind] = 1
                matrix[collide_partner_ind,index] = 1
            # collides with 2 balls
        elif num_collisions[index] > 1:
            # # hold indices of other balls being collided with
            # collide_partner_ind_vec = np.array([])
            # # consider all possible balls in collision matrix
            # for colision_index in range(len(num_collisions)):
            #     # get indices of balls colliding with current ball
            #     if(collision_matrix[index,colision_index] != 0):
            #         # if colliding, add to indices vector
            #         collide_partner_ind_vec = np.append(collide_partner_ind_vec, colision_index)  # Index of ball that is being collided with
            # # Momentum of current ball
            # p0a = p0_matrix[index,:] 
            # # find unit vector perpendicular to first colliding ball
            # perp_unit_vec1 = collision_vectors[index, collide_partner_ind_vec[0]] # Pulling first unit vector pointing to collision location for ball of interest
            # perp_unit_vec1 = perp_unit_vec[0, 0] 
            # perp_unit_vec2 = collision_vectors[index, collide_partner_ind_vec[1]] # Pulling unit second vector pointing to collision location for ball of interest
            # perp_unit_vec2 = perp_unit_vec[0, 0] 
            # perp_unit_vec = perp_unit_vec1 + perp_unit_vec2 / np.linalg.norm(perp_unit_vec1 + perp_unit_vec2)
            # p0b = p0_matrix[collide_partner_ind_vec[0],:] + p0_matrix[collide_partner_ind_vec[1],:]
            # p0a_perp = np.dot(p0a, perp_unit_vec) * perp_unit_vec# Projecting momentum in direction of collision for ball of interest
            # p0b_perp = np.dot(p0b, perp_unit_vec) * perp_unit_vec
            # mb_avg = (m_vec[collide_partner_ind_vec[0]] + m_vec[collide_partner_ind_vec[1]]) * 0.5

            # par_unit_vec, par_unit_vec2 = find_paralell_unit_vecs(perp_unit_vec, 0.0000001)

            # p0a_par1 = np.dot(par_unit_vec,p0a) * par_unit_vec # p0a projection into paralell dir 1 (conserved momentum)
            # p0a_par2 = np.dot(par_unit_vec2,p0a) * par_unit_vec2 # p0a projection into paralell dir 2 (conserved momentum)

            # #p0b_par1 = np.dot(par_unit_vec,p0b) * par_unit_vec # p0b projection into paralell dir 1 (conserved momentum)
            # #p0b_par2 = np.dot(par_unit_vec2,p0b) * par_unit_vec2 # p0b projection into paralell dir 2 (conserved momentum)

            # pfa_mag = (-e*mb_avg * np.linalg.norm(p0a_perp) + m_vec[index] * (np.linalg.norm(p0a_perp)+np.linalg.norm(p0b_perp) + e*np.linalg.norm(p0b_perp))) / (m_vec[index] + mb_avg)
            # pfa = pfa_mag * perp_unit_vec + p0a_par1 + p0a_par2
            # if abs(np.dot(p0a, perp_unit_vec) - np.dot(p0b, perp_unit_vec)) > epsilon:
            #     dp_vec[index] = pfa - p0a
            #     # dp_vec[collide_partner_ind_vec[0]] = p0a - pfa
            #     matrix = np.eye(len(num_collisions))
                
            # else:
            #     dp_vec[index] = np.array([0,0,0])
            #     dp_vec[collide_partner_ind_vec[0]] = np.array([0,0,0])
            #     dp_vec[collide_partner_ind_vec[1]] = np.array([0,0,0])
            #     matrix = np.zeros(len(num_collisions),len(num_collisions))
            #     matrix[index,collide_partner_ind_vec[0]] = 1
            #     matrix[collide_partner_ind_vec[0],index] = 1
            #     matrix[index,collide_partner_ind_vec[1]] = 1
            #     matrix[collide_partner_ind_vec[1],index] = 1
            # #dp_vec[collide_partner_ind] = p0a - pfa
            # break

        # general case, ignore 3
            # hold indices of other balls being collided with
            collide_partner_ind_vec = np.array([])
            # consider all possible balls in collision matrix
            for colision_index in range(len(num_collisions)):
                # get indices of balls colliding with current ball
                if(collision_matrix[index,colision_index] != 0):
                    # if colliding, add to indices vector
                    collide_partner_ind_vec = np.append(collide_partner_ind_vec, colision_index)  # Index of ball that is being collided with
            # Momentum of current ball
            p0a = p0_matrix[index,:] 
            # find unit vector perpendicular to first colliding ball

            # net unit vector
            perp_unit_vec = np.array([0.0, 0.0, 0.0])
            for i in range(len(collide_partner_ind_vec)):
                # get collision partner i
                perp_unit_veci = collision_vectors[index, int(collide_partner_ind_vec[i])] # Pulling first unit vector pointing to collision location for ball of interest
                # for some reason this has to be done
                #print(perp_unit_veci)
                #perp_unit_veci = perp_unit_veci[0, 0]
                perp_unit_vec += perp_unit_veci
            # normalize net perp vector
            perp_unit_vec = perp_unit_vec / np.linalg.norm(perp_unit_vec)

            # perp_unit_vec2 = collision_vectors[index, collide_partner_ind_vec[1]] # Pulling unit second vector pointing to collision location for ball of interest
            # perp_unit_vec2 = perp_unit_vec[0, 0] 
            # perp_unit_vec = perp_unit_vec1 + perp_unit_vec2 / np.linalg.norm(perp_unit_vec1 + perp_unit_vec2)
            
            # total momentum of colliding balls
            p0b = np.array([0.0, 0.0, 0.0])
            for i in range(len(collide_partner_ind_vec)):
                p0b += p0_matrix[int(collide_partner_ind_vec[i])]
            
            #p0b = p0_matrix[collide_partner_ind_vec[0],:] + p0_matrix[collide_partner_ind_vec[1],:]
            
            # perpendicular projection of momentum of current ball
            p0a_perp = np.dot(p0a, perp_unit_vec) * perp_unit_vec# Projecting momentum in direction of collision for ball of interest
            p0b_perp = np.dot(p0b, perp_unit_vec) * perp_unit_vec
            
            # average mass of b
            mb_avg = 0
            for i in range(len(collide_partner_ind_vec)):
                mb_avg += m_vec[int(collide_partner_ind_vec[i])]
            mb_avg /= len(num_collisions)
            #mb_avg = (m_vec[collide_partner_ind_vec[0]] + m_vec[collide_partner_ind_vec[1]]) * 0.5

            # parallel vectors
            par_unit_vec, par_unit_vec2 = find_paralell_unit_vecs(perp_unit_vec, 0.0000001)

            p0a_par1 = np.dot(par_unit_vec,p0a) * par_unit_vec # p0a projection into paralell dir 1 (conserved momentum)
            p0a_par2 = np.dot(par_unit_vec2,p0a) * par_unit_vec2 # p0a projection into paralell dir 2 (conserved momentum)

            #p0b_par1 = np.dot(par_unit_vec,p0b) * par_unit_vec # p0b projection into paralell dir 1 (conserved momentum)
            #p0b_par2 = np.dot(par_unit_vec2,p0b) * par_unit_vec2 # p0b projection into paralell dir 2 (conserved momentum)

            # compute final momentum for current ball, p final a
            pfa_mag = (-e*mb_avg * np.linalg.norm(p0a_perp) + m_vec[index] * (np.linalg.norm(p0a_perp)+np.linalg.norm(p0b_perp) + e*np.linalg.norm(p0b_perp))) / (m_vec[index] + mb_avg)
            pfa = pfa_mag * perp_unit_vec + p0a_par1 + p0a_par2
            if abs(np.dot(p0a, perp_unit_vec) - np.dot(p0b, perp_unit_vec)) > epsilon:
                dp_vec[index] = pfa - p0a
                # dp_vec[collide_partner_ind_vec[0]] = p0a - pfa
                matrix = np.eye(len(num_collisions))
                
            else:
                # stop balls from going through each other
                dp_vec[index] = np.array([0,0,0])
                for i in range(len(collide_partner_ind_vec)):
                    dp_vec[collide_partner_ind_vec[i]] = np.array([0,0,0])
                #dp_vec[collide_partner_ind_vec[1]] = np.array([0,0,0])
                matrix = np.zeros(len(num_collisions),len(num_collisions))

                for i in range(len(collide_partner_ind_vec)):
                    matrix[index,collide_partner_ind_vec[i]] = 1
                    matrix[collide_partner_ind_vec[i],index] = 1
                #matrix[index,collide_partner_ind_vec[1]] = 1
                #matrix[collide_partner_ind_vec[1],index] = 1
            #dp_vec[collide_partner_ind] = p0a - pfa

        else:
            dp_vec[index] = np.array([0,0,0])
            matrix = np.eye(len(num_collisions))
    # debug
    # add wall collision to ball collision
    dp_vec = dp_vec + wall_dp_vec
    return(dp_vec, matrix)

def find_paralell_unit_vecs(perp_unit_vec, error_val):
    while_index = 0
    error = 1
    while error > error_val:
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

        return par_unit_vec, par_unit_vec2

def gravitation(x_vec, y_vec, z_vec, m_vec):
    # gets all pairs of mass products excluding with self
    #adj_matrix = np.outer(m_vec, m_vec) - np.eye(len(m_vec)) @ m_vec**2



    # matrix of r^2 values, same shape as adj
    r_mat = np.zeros((len(m_vec), len(m_vec)))
    # get the r^2 matrix
    inv_x_mat = np.zeros(r_mat.shape)
    inv_y_mat = np.zeros(r_mat.shape)
    inv_z_mat = np.zeros(r_mat.shape)
    for i in range(len(r_mat)):
        for j in range(len(r_mat)):
            if i != j:
                r_veci = np.array([x_vec[i], y_vec[i], z_vec[i]])
                r_vecj = np.array([x_vec[j], y_vec[j], z_vec[j]])
                r = r_vecj - r_veci
                # get 1/r^2 basically
                inv_norm = (m_vec[j] * m_vec[i]) / (np.dot(r, r))

                inv_norm_x = inv_norm * (x_vec[j] - x_vec[i]) / np.linalg.norm(r)
                inv_norm_y = inv_norm * (y_vec[j] - y_vec[i]) / np.linalg.norm(r)
                inv_norm_z = inv_norm * (z_vec[j] - z_vec[i]) / np.linalg.norm(r)

                inv_x_mat[i, j] = inv_norm_x
                inv_y_mat[i, j] = inv_norm_y
                inv_z_mat[i, j] = inv_norm_z


                r_mat[i, j] = inv_norm

    # get a column vector of ones
    ones_vec = np.ones((len(r_mat), 1))

    inv_x_vec = inv_x_mat @ ones_vec
    inv_y_vec = inv_y_mat @ ones_vec
    inv_z_vec = inv_z_mat @ ones_vec

    #r_vec = r_mat @ ones_vec

    grav_x = inv_x_vec
    grav_y = inv_y_vec
    grav_z = inv_z_vec
    #grav = adj_vec * r_vec

    return grav_x, grav_y, grav_z


def electrostatic(x_vec, y_vec, z_vec, q_vec):
    # matrix of r^2 values, same shape as adj
    r_mat = np.zeros((len(q_vec), len(q_vec)))
    # get the r^2 matrix
    inv_x_mat = np.zeros(r_mat.shape)
    inv_y_mat = np.zeros(r_mat.shape)
    inv_z_mat = np.zeros(r_mat.shape)
    for i in range(len(r_mat)):
        for j in range(len(r_mat)):
            if i != j:
                r_veci = np.array([x_vec[i], y_vec[i], z_vec[i]])
                r_vecj = np.array([x_vec[j], y_vec[j], z_vec[j]])
                r = r_vecj - r_veci
                # get 1/r^2 basically
                inv_norm = (q_vec[j] * q_vec[i]) / (np.dot(r, r))

                inv_norm_x = inv_norm * (x_vec[j] - x_vec[i]) / np.linalg.norm(r)
                inv_norm_y = inv_norm * (y_vec[j] - y_vec[i]) / np.linalg.norm(r)
                inv_norm_z = inv_norm * (z_vec[j] - z_vec[i]) / np.linalg.norm(r)

                inv_x_mat[i, j] = inv_norm_x
                inv_y_mat[i, j] = inv_norm_y
                inv_z_mat[i, j] = inv_norm_z


                r_mat[i, j] = inv_norm

    # get a column vector of ones
    ones_vec = np.ones((len(r_mat), 1))

    inv_x_vec = inv_x_mat @ ones_vec
    inv_y_vec = inv_y_mat @ ones_vec
    inv_z_vec = inv_z_mat @ ones_vec

    #r_vec = r_mat @ ones_vec

    elec_x = inv_x_vec
    elec_y = inv_y_vec
    elec_z = inv_z_vec
    #grav = adj_vec * r_vec

    return elec_x, elec_y, elec_z

def magnetism(x_vec, y_vec, z_vec, q_vec, px_vec, py_vec, pz_vec, m_vec):
    # matrix of r^2 values, same shape as adj
    r_mat = np.zeros((len(q_vec), len(q_vec)))
    # get the r^2 matrix
    x_mat = np.zeros(r_mat.shape)
    y_mat = np.zeros(r_mat.shape)
    z_mat = np.zeros(r_mat.shape)
    for i in range(len(r_mat)):
        for j in range(len(r_mat)):
            if i != j:
                r_veci = np.array([x_vec[i], y_vec[i], z_vec[i]])
                r_vecj = np.array([x_vec[j], y_vec[j], z_vec[j]])
                r = r_vecj - r_veci
                # get 1/r^2 basically
                inv_norm = (q_vec[j] * q_vec[i]) / (np.dot(r, r))

                # velocity of body i
                vx_i = px_vec[i] / m_vec[i]
                vy_i = py_vec[i] / m_vec[i]
                vz_i = pz_vec[i] / m_vec[i]

                # velocity of body j
                vx_j = px_vec[j] / m_vec[j]
                vy_j = py_vec[j] / m_vec[j]
                vz_j = pz_vec[j] / m_vec[j]

                # displacement from i to j
                dx = r[0]
                dy = r[1]
                dz = r[2]

                x_comp = (vy_j * (vx_i * dy - vy_i * dx) - vz_j * (vz_i * dx - vx_i * dz)) / np.linalg.norm(r)
                y_comp = (vz_j * (vy_i * dz - vz_i * dy) - vx_j * (vx_i * dy - vy_i * dx)) / np.linalg.norm(r)
                z_comp = (vx_j * (vz_i * dx - vx_i * dz) - vy_j * (vy_i * dz - vz_i * dy)) / np.linalg.norm(r)

                x_comp *= inv_norm
                y_comp *= inv_norm
                z_comp *= inv_norm

                x_mat[i, j] = x_comp
                y_mat[i, j] = y_comp
                z_mat[i, j] = z_comp
    
    # get a column vector of ones
    ones_vec = np.ones((len(r_mat), 1))
    mag_x = x_mat @ ones_vec
    mag_y = y_mat @ ones_vec
    mag_z = z_mat @ ones_vec

    return mag_x, mag_y, mag_z




# dynamics(np.array([[1,2,3,5,3,2,1],
#                   [4,5,6,5,3,2,1],
#                   [7,8,9,5,3,2,1]]), 0.001, "x+y",np.zeros((3,3)))