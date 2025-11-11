import numpy as np
import forward_kinematics as fk


def IK_analytical(x,y,z,theta,degrees=False):
    '''
    function using analytical inverse kinematics to get sets of joint angles that will result in a desired end effector position/orientation. for most goal points there are 4 solutions that achieve the goal (elbow up/down for the revolute portion and elbow up/down for the prismatic joint) 
    '''
    check_workspace(x,y,z,theta,degrees)
    if degrees: # if the input is in degrees then convert it to radians
        theta = np.radians(theta)
    theta = theta - np.pi/2 # rotate target angle -90 deg to match the x/y-axis orientation of link frames

    # theta_0 calculation
    d_0 = z - fk.T_B0[2,3] - (fk.geom[1]['offset']+fk.geom[2]['offset']+fk.geom[3]['offset'])
    r_plinkage = np.sqrt(d_0**2 + fk.geom[0]['ht_diff']**2) # cartesian distance from motor 0 axis to frame 1 origin
    beta_pris = np.arctan2(-fk.geom[0]['ht_diff'], d_0)
    psi_pris = np.arccos((fk.geom[0]['l_crank']**2 + r_plinkage**2 - fk.geom[0]['l_crod']**2)/ \
            (2*r_plinkage*fk.geom[0]['l_crank']))
    
    theta_0_elbup = beta_pris + psi_pris
    theta_0_elbdn = beta_pris - psi_pris
    
    x_wrist = y-fk.geom[4]['l_eff']*np.cos(theta)
    y_wrist = -x-fk.geom[4]['l_eff']*np.sin(theta)
    r_wrist = np.sqrt(x_wrist**2 + y_wrist**2) # cartesian distance from top of tower to wrist axis

    # theta_1 calculation
    beta = np.arctan2(y_wrist,x_wrist)
    psi = np.arccos((fk.geom[2]['l_link']**2+r_wrist**2-fk.geom[3]['l_link']**2)/ \
            (2*fk.geom[2]['l_link']*r_wrist))
    
    theta_1_elbup = beta + psi
    theta_1_elbdn = beta - psi

    # theta_2 calculation
    theta_2_elbdn = np.arccos((r_wrist**2-fk.geom[2]['l_link']**2-fk.geom[3]['l_link']**2)/ \
            (2*fk.geom[2]['l_link']*fk.geom[3]['l_link']))
    theta_2_elbup = 2*np.pi - theta_2_elbdn # the "elbow up" solution should be when theta_2 is a reflex angle (i think)

    # theta_3 calculation
    theta_3_elbup = theta - theta_1_elbup - theta_2_elbup
    theta_3_elbdn = theta - theta_1_elbdn - theta_2_elbdn

    sols = np.array([
        [theta_0_elbup,theta_0_elbdn],
        [theta_1_elbup,theta_1_elbdn],
        [theta_2_elbup,theta_2_elbdn],
        [theta_3_elbup,theta_3_elbdn]])
    
    if degrees:
        return np.degrees(sols)
    else:
        return sols

def IK_numerical(x,y,z,theta,guess,degrees=False,tol=1e-3,maxiters=100):
    '''
    function using numerical inverse kinematics to find a set of joint positions that accomplishes a desired end effector position, starting from a guess vector of joint angles and using newton-raphson iteration to converge toward a solution
    '''
    raise NotImplementedError('oops')

    check_workspace(x,y,z,theta,degrees)

    J = fk.jacobian(*guess)

def check_workspace(x,y,z,theta,degrees=False):
    '''
    utility function to check if a target point is inside the robot's workspace. raises ValueError if not
    '''
    if z < fk.lims['z_min'] or z > fk.lims['z_max']:
        raise ValueError('Target z-coordinate outside of workspace.')

    if degrees: # if the input is in degrees then convert it to radians
        theta = np.radians(theta)
    
    theta = theta - np.pi/2 # rotate target angle -90 deg to match the x/y-axis orientation of link frames
    x_wrist = y-fk.geom[4]['l_eff']*np.cos(theta)
    y_wrist = -x-fk.geom[4]['l_eff']*np.sin(theta)
    r_wrist = np.sqrt(x_wrist**2+y_wrist**2)
    if r_wrist > fk.lims['r_max']:
        raise ValueError('Target x, y, theta combination outside of workspace.')
    