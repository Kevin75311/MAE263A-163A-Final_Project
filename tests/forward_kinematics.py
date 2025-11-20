import numpy as np

geom = [0,0,0,0,0] # store some link lengths and other offsets that haven't yet been determined
# link/joint 0 (prismatic) geometric properies:
geom[0] = {
    'l_crank': 0.06, # [m]
    'l_crod': 0.15, # [m]
    'ht_diff': 0.07, # [m]
}
# link/joint 1 (revolute) geometric properties:
geom[1] = {
    'h_tower': 0.1, # [m]
    'offset': -0.01, # [m]
}
# link/joint 2 (revolute) geometric properties:
geom[2] = {
    'l_link': 0.08, # [m]
    'offset': -0.01, # [m]
}
# link/joint 3 (revolute) geometric properties:
geom[3] = {
    'l_link': 0.08, # [m]
    'offset': 0.01, # [m]
}
# end effector geometric properties:
geom[4] = {
    'l_eff': 0.02, # [m]
}

T_B0 = np.array([ # static transform expressing the position of frame {0} in frame {B}, a base frame with y-axis up and 0,0,0 at the center of the reachable workspace
        [0.,-1., 0., 0.],
        [1., 0., 0., -geom[1]['h_tower']],
        [0., 0., 1., -((np.sqrt((geom[0]['l_crod']-geom[0]['l_crank'])**2 - geom[0]['ht_diff']**2) + np.sqrt((geom[0]['l_crod']+geom[0]['l_crank'])**2 - geom[0]['ht_diff']**2))/2 + geom[1]['offset']+geom[2]['offset']+geom[3]['offset'])],
        [0., 0., 0., 1.]
    ])

lims = {
    'z_min':(np.sqrt((geom[0]['l_crod']-geom[0]['l_crank'])**2 - geom[0]['ht_diff']**2) - np.sqrt((geom[0]['l_crod']+geom[0]['l_crank'])**2 - geom[0]['ht_diff']**2))/2,
    'z_max':(np.sqrt((geom[0]['l_crod']+geom[0]['l_crank'])**2 - geom[0]['ht_diff']**2) - np.sqrt((geom[0]['l_crod']-geom[0]['l_crank'])**2 - geom[0]['ht_diff']**2))/2,
    'r_max':np.sqrt(geom[2]['l_link']**2 + geom[3]['l_link']**2),
}

T_4f = np.array([ # static transform expressing the position of end effector tip frame {f} in end effector joint frame {4}
        [1., 0., 0., geom[4]['l_eff']],
        [0., 1., 0., 0.],
        [0., 0., 1., 0.],
        [0., 0., 0., 1.]
    ])

def get_T_01(theta_0, degrees=False):
    if degrees: theta_0 = np.radians(theta_0) # if the input is in degrees then convert it to radians
    c0 = np.cos(theta_0)
    s0 = np.sin(theta_0)
    d_0 = geom[0]['l_crank']*c0 + \
            np.sqrt(
                geom[0]['l_crod']**2 - \
                (geom[0]['ht_diff'] + geom[0]['l_crank']*s0)**2
            )
    T_01 = np.array([
        [1., 0., 0., 0,],
        [0., 1., 0., 0,],
        [0., 0., 1., d_0],
        [0., 0., 0., 1.]
    ])
    return T_01

def get_T_12(theta_1, degrees=False):
    if degrees: theta_1 = np.radians(theta_1) # if the input is in degrees then convert it to radians
    c1 = np.cos(theta_1)
    s1 = np.sin(theta_1)
    T_12 = np.array([
        [c1,-s1, 0., geom[1]['h_tower']],
        [s1, c1, 0., 0.],
        [0., 0., 1., geom[1]['offset']],
        [0., 0., 0., 1.]
    ])
    return T_12

def get_T_23(theta_2, degrees=False):
    if degrees: theta_2 = np.radians(theta_2) # if the input is in degrees then convert it to radians
    c2 = np.cos(theta_2)
    s2 = np.sin(theta_2)
    T_23 = np.array([
        [c2,-s2, 0., geom[2]['l_link']],
        [s2, c2, 0., 0.],
        [0., 0., 1., geom[2]['offset']],
        [0., 0., 0., 1.]
    ])
    return T_23

def get_T_34(theta_3, degrees=False):
    if degrees: theta_3 = np.radians(theta_3) # if the input is in degrees then convert it to radians
    c3 = np.cos(theta_3)
    s3 = np.sin(theta_3)
    T_34 = np.array([
        [c3,-s3, 0., geom[3]['l_link']],
        [s3, c3, 0., 0.],
        [0., 0., 1., geom[3]['offset']],
        [0., 0., 0., 1.]
    ])
    return T_34

def get_T_Bf(theta_0, theta_1, theta_2, theta_3, degrees=False):
    T_01 = get_T_01(theta_0,degrees)
    T_12 = get_T_12(theta_1,degrees)
    T_23 = get_T_23(theta_2,degrees)
    T_34 = get_T_34(theta_3,degrees)
    return T_B0 @ T_01 @ T_12 @ T_23 @ T_34 @ T_4f

def plot_manipulator(theta_0, theta_1, theta_2, theta_3, ax, degrees=False):
    T_01 = get_T_01(theta_0,degrees)
    T_12 = get_T_12(theta_1,degrees)
    T_23 = get_T_23(theta_2,degrees)
    T_34 = get_T_34(theta_3,degrees)
    T_B1 = T_B0 @ T_01
    T_B2 = T_B1 @ T_12
    T_B3 = T_B2 @ T_23
    T_B4 = T_B3 @ T_34
    T_Bf = T_B4 @ T_4f
    if degrees: theta_0 = np.degrees(theta_0)
    q0 = ax.quiver(*T_B0[:3,3],*(T_B1[:3,3]-T_B0[:3,3]),arrow_length_ratio=0.1,color='red',linestyle='--') # link 0 (prismatic)
    vec_ht_diff = np.array([0,geom[0]['ht_diff'],0])
    vec_crank = np.array([0,geom[0]['l_crank']*np.sin(theta_0),geom[0]['l_crank']*np.cos(theta_0)])
    q0c = ax.quiver(*(T_B0[:3,3]+vec_ht_diff),*vec_crank,arrow_length_ratio=0.1,color='red') # crank arm
    q0r = ax.quiver(*(T_B0[:3,3]+vec_ht_diff+vec_crank),*(T_B1[:3,3]-T_B0[:3,3]-vec_crank-vec_ht_diff),arrow_length_ratio=0.1,color='red') # connecting rod
    q1 = ax.quiver(*T_B1[:3,3],*(T_B2[:3,3]-T_B1[:3,3]),arrow_length_ratio=0.1,color='blue') # link 1 (tower)
    q2 = ax.quiver(*T_B2[:3,3],*(T_B3[:3,3]-T_B2[:3,3]),arrow_length_ratio=0.1,color='green') # link 2 (arm 1)
    q3 = ax.quiver(*T_B3[:3,3],*(T_B4[:3,3]-T_B3[:3,3]),arrow_length_ratio=0.1,color='orange') # link 3 (arm 2)
    q4 = ax.quiver(*T_B4[:3,3],*(T_Bf[:3,3]-T_B4[:3,3]),arrow_length_ratio=0.1,color='black') # link 4 (end effector)
    return T_Bf[:3,3], [q0,q0c,q0r,q1,q2,q3,q4]

def jacobian(theta_0, theta_1, theta_2, theta_3, degrees=False):
    '''
    calculates the space jacobian of a manipulator given its configuration, which relates the task space (x,y,z,t) to joint space (t0,t1,t2,t3)
    
               [[dx/dt0  dx/dt1  dx/dt2  dx/dt3]
    Jacobian =  [dy/dt0  dy/dt1  dy/dt2  dy/dt3]
                [dz/dt0  dz/dt1  dz/dt2  dx/dt3]
                [dt/dt0  dt/dt1  dt/dt2  dt/dt3]]
    '''
    z_ax = np.array([0.,0.,1.])
    T_01 = get_T_01(theta_0,degrees)
    T_12 = get_T_12(theta_1,degrees)
    T_23 = get_T_23(theta_2,degrees)
    T_34 = get_T_34(theta_3,degrees)
    T_B2 = T_B0 @ T_01 @ T_12
    T_B3 = T_B2 @ T_23
    T_B4 = T_B3 @ T_34
    T_Bf = T_B4 @ T_4f

    if degrees: theta_0 = np.radians(theta_0) # if the input is in degrees then convert it to radians
    s0 = np.sin(theta_0)
    c0 = np.cos(theta_0)

    J = np.zeros((4,4))
    dz_dt0 = -geom[0]['l_crank']*s0 - \
            geom[0]['l_crank']*c0*(geom[0]['ht_diff'] + geom[0]['l_crank']*s0)/ \
            np.sqrt(
                (geom[0]['l_crod']**2 - (geom[0]['ht_diff'] + geom[0]['l_crank']*s0)**2))
    J[0] = np.array([0.,0.,dz_dt0,0.])
    J[1] = np.append(np.cross(
        z_ax,
        T_Bf[:3,3]-T_B2[:3,3]
        ),1)
    J[2] = np.append(np.cross(
        z_ax,
        T_Bf[:3,3]-T_B3[:3,3]
        ),1)
    J[3] = np.append(np.cross(
        z_ax,
        T_Bf[:3,3]-T_B4[:3,3]
        ),1)
    
    return J

def T_inv(T_mat):
    '''returns the matrix inverse of a homogenous transform while using less computational power than directly taking the inverse (hopefully)'''
    rot_mat = T_mat[:3,:3]
    disp_vec = T_mat[:3,3]
    return np.vstack(
        [np.hstack([rot_mat.T, -rot_mat @ disp_vec[:,None]]),
         np.array([0.,0.,0.,1.])])
