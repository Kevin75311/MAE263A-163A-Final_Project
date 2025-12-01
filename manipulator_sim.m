% MAE 263A Project
% Simulation
clc
cla

% Geometry of the manipulator
geom.lcrank = 0.1524; % length of crank arm
geom.lcrod = 0.254; % length of connecting rod
geom.scht = -0.00875; % height difference between crank axis and link axle
geom.htower = 0.1306; % height of the link 1 tower
geom.l2 = 0.1164; % link 2 length
geom.l3 = 0.0974; % link 3 length
geom.leff = 0.00805; % end effector length
geom.offset = 0.041; % z-axis offset between links

% static transform expressing the position of frame {0} in frame {B}, a 
% base frame with y-axis up and 0,0,0 at the center of the reachable 
% workspace (matching solidworks conventions)
wkspace_offset_z = -((sqrt((geom.lcrod-geom.lcrank)^2 - geom.scht^2) + ...
    sqrt((geom.lcrod+geom.lcrank)^2 - geom.scht^2))/2 + 3*geom.offset);
T_B0 = [0 -1  0  0 
        1  0  0 -geom.htower
        0  0  1  wkspace_offset_z
        0  0  0  1];

% static transform expressing the position of end effector tip frame {f} in
% end effector joint frame {4}
T_4f = [1 0 0 geom.leff
        0 1 0 0
        0 0 1 0
        0 0 0 1];

% transform to reorient axes to be z-up for plotting
T_PB = [-1 0 0 0 
         0 0 1 0
         0 1 0 0
         0 0 0 1];

T_P0 = T_PB*T_B0;

t = linspace(0,2*pi,100);
x = sin(t)*0.1;
y = cos(t)*0.08;
z = cos(t)*0.07;
c = [0.1 0.2 0.1]; % color of the links

hold on
for i=1:length(t)
    pos = [x(i); y(i); z(i)];
    % calculate the inverse kinematics solution to get joint angles
    angles = IK_analytical(geom,pos,0);
    % find link positions and end effector location from forward kinematics
    [~, ~,T_01,T_12,T_23,T_34,T_Bf] = FK(geom, angles, T_B0, T_4f);

    % remove previously plotted objects from the figure
    delete(findobj(type='patch'));
    delete(findobj(type='line'));

    % plot the desired path of the end effector
    plot3(-x(1:i),z(1:i),y(1:i),'b');

    % plot the links of the manipulator
    plot_link(T_P0, 0.1,0.01,0.03,0.03,c);
    plot_link(T_P0*T_01,geom.htower,0.01,0.03,geom.offset,c);
    plot_link(T_P0*T_01,0.1,-0.07,0.03,T_01(3,4),c);
    plot_link(T_P0*T_01*T_12,geom.l2,0.01,0.03,geom.offset,c);
    plot_link(T_P0*T_01*T_12*T_23,geom.l3,0.01,0.03,geom.offset,c);
    plot_link(T_P0*T_01*T_12*T_23*T_34,geom.leff,0.01,0.02,geom.offset,c);
    plot_link(T_PB*T_Bf,0,0,0,0,[0.1 0.1 0.1]);
    if i==1
        % pause plotting at the first timestep to let me reorient view
        axis equal;
        axis([-0.15 0.15 -0.5 0.2 -0.15 0.15]);
        view(140,20);
        title('animation paused, press any key to resume')
        pause
        title('')
    end
    % pause between timesteps
    pause(0.005)
end

function plot_link(T, xplus, xmin, ysize, zsize, c)
    framepos = T(1:3,4);

    % plot the coordinate frame
    mag = 0.01;
    plot3(framepos(1)+[0 T(1,1)]*mag, ...
          framepos(2)+[0 T(2,1)]*mag, ...
          framepos(3)+[0 T(3,1)]*mag,'r',LineWidth=1.5); % x
    plot3(framepos(1)+[0 T(1,2)]*mag, ...
          framepos(2)+[0 T(2,2)]*mag, ...
          framepos(3)+[0 T(3,2)]*mag,'g',LineWidth=1.5); % y
    plot3(framepos(1)+[0 T(1,3)]*mag*1.2, ...
          framepos(2)+[0 T(2,3)]*mag*1.2, ...
          framepos(3)+[0 T(3,3)]*mag*1.2,'b',LineWidth=2); % z

    % vertices of a rectangular prism representation of the link
    xs = [-xmin xplus];
    ys = [-ysize/2 ysize/2];
    zs = [-zsize 0];
    verts = T * [xs(1) xs(2) xs(1) xs(2) xs(1) xs(2) xs(1) xs(2)
                 ys(1) ys(1) ys(2) ys(2) ys(1) ys(1) ys(2) ys(2)
                 zs(1) zs(1) zs(1) zs(1) zs(2) zs(2) zs(2) zs(2)
                 1     1     1     1     1     1     1     1];
    face_idx = [1 5 1 1 2 4
                2 6 3 2 4 8
                4 8 7 6 8 7
                3 7 5 5 6 3];
    xpts = verts(1,:);
    ypts = verts(2,:);
    zpts = verts(3,:);
    fill3(xpts(face_idx), ypts(face_idx), zpts(face_idx), c);
end

function [pos, theta, T_01,T_12,T_23,T_34,T_Bf] = FK(geom,angles,T_B0,T_4f)
    % prismatic joint transform
    c0 = cos(angles(1));
    s0 = sin(angles(1));
    d_0 = geom.lcrank*c0 + sqrt( ...
                geom.lcrod^2 - ...
                (geom.scht + geom.lcrank*s0)^2);
    T_01 = [1 0 0 0
            0 1 0 0
            0 0 1 d_0
            0 0 0 1];

    % revolute joint transforms are all really simple and just have
    % different length parameters
    c1 = cos(angles(2));
    s1 = sin(angles(2));
    T_12 = [c1 -s1  0  geom.htower
            s1  c1  0  0
            0   0   1  geom.offset
            0   0   0  1];

    c2 = cos(angles(3));
    s2 = sin(angles(3));
    T_23 = [c2 -s2  0  geom.l2
            s2  c2  0  0
            0   0   1  geom.offset
            0   0   0  1];

    c3 = cos(angles(4));
    s3 = sin(angles(4));
    T_34 = [c3 -s3  0  geom.l3
            s3  c3  0  0
            0   0   1  geom.offset
            0   0   0  1];

    % combining all the transforms together
    T_Bf = T_B0*T_01*T_12*T_23*T_34*T_4f;

    pos = T_Bf(1:3,4);
    theta = atan2(T_Bf(2,1), T_Bf(1,1));
end

function [angles] = IK_analytical(geom,pos,theta)
    x = pos(1); y = pos(2); z = pos(3);

    % theta_0 (prismatic joint's driving crank angle) computation
    wkspace_offset_z = (sqrt((geom.lcrod-geom.lcrank)^2 - geom.scht^2) + ...
        sqrt((geom.lcrod+geom.lcrank)^2 - geom.scht^2))/2;
    d_0 = z + wkspace_offset_z;
    r_plinkage = sqrt(d_0^2 + geom.scht^2);
    beta_pris = atan2(-geom.scht, d_0);

    % law of cosines to get psi (using arccos instead of atan2 works fine)
    psi_pris = acos( ...
        (geom.lcrank^2 + r_plinkage^2 - geom.lcrod^2) / ...
        (2*r_plinkage*geom.lcrank));

    theta_0_elbup = beta_pris + psi_pris; % we decided to always use elbup
    theta_0_elbdn = beta_pris - psi_pris;


    % rotate target angle -90 deg to match the x/y-axis orientation of link
    % frames (rather than that of global/base frame)
    theta_local = theta - pi/2;
    % wrist coordinates in tower frame (end effector DoF controls angle)
    x_wrist = y - geom.leff * cos(theta_local);
    y_wrist = -x - geom.leff * sin(theta_local);
    r_wrist = sqrt(x_wrist^2 + y_wrist^2);

    % theta_1
    beta = atan2(y_wrist, x_wrist);
    psi = acos((geom.l2^2 + r_wrist^2 - geom.l3^2) / ...
        (2 * geom.l2 * r_wrist));
    theta_1_elbup = beta + psi;
    theta_1_elbdn = beta - psi;

    % theta_2 ("elbow")
    theta_2_elbdn = acos((r_wrist^2 - geom.l2^2 - geom.l3^2) / ...
        (2.0 * geom.l2 * geom.l3));
    theta_2_elbup = 2.0 * pi - theta_2_elbdn;

    % theta_3 to satisfy orientation
    theta_3_elbup = theta_local - theta_1_elbup - theta_2_elbup;
    theta_3_elbdn = theta_local - theta_1_elbdn - theta_2_elbdn;

    angles = [theta_0_elbup
              theta_1_elbdn
              theta_2_elbdn
              theta_3_elbdn];
end