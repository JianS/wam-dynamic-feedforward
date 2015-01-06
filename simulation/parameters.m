%% Setting object parameters

% Initial condition - object
x0 = -1.2; y0 = 0.0; z0 = 0.2;
Vx = 1.5; Vy = 0.0; Vz = 4;
g = -10;
ax = -.5; ay = -0.0; az = g;

theta0 = -pi/6;
omega = 6;%2*pi;
RA = [0 1 0]; %rotation axis;

q0 = quaternion.angleaxis(theta0, RA)*quaternion([1; 0;0;0]);

% Initial condition - hand
xc = 0; yc = 0; zc = 0; % robot origin
x0_hand = -0.4; y0_hand = 0.0; z0_hand = 0.5;

theta0_hand = 0;
RA_hand = [0 1 0]; %rotation axis;
q0_hand = quaternion.angleaxis(theta0_hand, RA_hand);

% robot limitation
V_limit_up = 3;
V_limit_down = -3;

a_limit_up = 7;
a_limit_down = -20;

torque_limit = .5;
force_limit = 2;

omega_hand = 10;

l_robot = 0.45;

% Work-space 
r = 0.85;
r_inner = 0.2;

% Object parameter

l_cube = 6*0.0254;
r_cube = 2^0.5*0.5*l_cube+0.01; % circle around the object define close rigion

m = 0.15; % mass of the object, in kg
I = m*l_cube^2/6; % moment of inertia - along y-axis


% hand geometry
l_hand = 12*0.0254;
h_hand = 0.0243;

t2apex = -Vz/az;
apex = [(x0 + Vx*t2apex + 0.5*ax*t2apex^2) (y0 + Vy*t2apex + 0.5*ay*t2apex^2) (z0 + Vz*t2apex + 0.5*az*t2apex^2)];

% time line
t_start = 0; t_end = 1.2; t_int = 0.002;
t = t_start:t_int:t_end;
n = size(t,2);


