import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import forward_kinematics as fk
import inverse_kinematics as ik
matplotlib.use('TkAgg')


# test 1: see if the forward kinematics works =========================================
# """
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-0.2,0.2)
ax.set_ylim(-0.2,0.2)
ax.set_zlim(-0.3,0.15)
# set axes labels
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')

positions = np.zeros((3,2000))
pts = ax.scatter(0,0,0)
artists = []

for t in range(positions.shape[1]):
    [artist.remove() for artist in artists] # clear data from previous iteration
    pts.remove()

    theta_0 = -np.pi*t/positions.shape[1] - np.arcsin(fk.geom[0]['ht_diff']/(fk.geom[0]['l_crank']+fk.geom[0]['l_crod']))# determine joint angles to hit different points within the workspace
    theta_1 = 20*np.pi*t/positions.shape[1]
    theta_2 = 90*np.pi*t/positions.shape[1]
    theta_3 = 50*np.pi*t/positions.shape[1]

    positions[:,t], artists = fk.plot_manipulator(theta_0, theta_1, theta_2, theta_3, ax) # plot the config of the arm
    pts = ax.scatter(positions[0,:t+1],positions[1,:t+1],positions[2,:t+1],c='b')   # plot all the points we've visited
    plt.pause(0.0005)
plt.show()

# """

# test 2: see if we can use inverse kinematics to draw something =======================================
"""
t_vec = np.linspace(0,2*np.pi,100)
x_vec = np.sin(t_vec)*0.1 # parametric eqns for an angled ellipsoid thingy
y_vec = np.cos(t_vec)*0.08
z_vec = np.cos(t_vec)*0.0707

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-0.2,0.2)
ax.set_ylim(-0.2,0.2)
ax.set_zlim(-0.3,0.15)
ax.scatter(x_vec,y_vec,z_vec)

positions = np.zeros((3,400))
reached_pts = ax.scatter(0,0,0)

artists = []
for i in range(t_vec.shape[0]):
    [artist.remove() for artist in artists] # clear data from previous iteration
    reached_pts.remove()
    ik_result = ik.IK_analytical(x_vec[i],y_vec[i],z_vec[i],np.arctan2(y_vec[i],x_vec[i]))
    positions[:,i], artists = fk.plot_manipulator(*ik_result[:,1].T,ax)
    reached_pts = ax.scatter(positions[0,:i+1],positions[1,:i+1],positions[2,:i+1],c='b')   # plot all the points we've visited
    plt.pause(0.01)

plt.show()

"""

# test 3: see if we can use numerical ik to draw something =======================================
#"""
t_vec = np.linspace(0,2*np.pi,100)
x_vec = np.sin(t_vec)*0.1 # parametric eqns for an angled ellipsoid thingy
y_vec = np.cos(t_vec)*0.08
z_vec = np.cos(t_vec)*0.0707

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-0.2,0.2)
ax.set_ylim(-0.2,0.2)
ax.set_zlim(-0.3,0.15)
ax.scatter(x_vec,y_vec,z_vec)

positions = np.zeros((3,400))
reached_pts = ax.scatter(0,0,0)
joint_angles = np.array([1.,1.,1.,1.]) # guess value for initial position (hopfully not a singularity?)

artists = []
for i in range(t_vec.shape[0]):
    [artist.remove() for artist in artists] # clear data from previous iteration
    reached_pts.remove()
    ik_result = ik.IK_numerical(x_vec[i],y_vec[i],z_vec[i],np.arctan2(y_vec[i],x_vec[i]),joint_angles)
    positions[:,i], artists = fk.plot_manipulator(*ik_result.T,ax)
    joint_angles = ik_result.copy()
    reached_pts = ax.scatter(positions[0,:i+1],positions[1,:i+1],positions[2,:i+1],c='b')   # plot all the points we've visited
    plt.pause(0.01)

plt.show()

#"""