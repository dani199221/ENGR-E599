% implementation of translation dynamics
function[b3d, f] = t_dynamics(xd, k_x, k_v, x, v, m, g, e3, vd, d_d_xd, R)

% calculate the tracking errors
ex = x - xd;
ev = v - vd;

% calculate b3d
b3d_nume = -k_x*ex - k_v*ev - m*g*e3 + m*d_d_xd; % 3x1
b3d = -(b3d_nume/norm(b3d_nume));

% calculate the force vector
f = -(b3d_nume).*(R*e3); % 3x1

end