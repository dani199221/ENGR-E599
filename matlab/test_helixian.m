% initial position of the quadrotor
x0 = [0,0,0];
v0 = [0,0,0];
R0 = eye(3);
omega0 = [0,0,0];

% quadrotor specific values
j = [0.0820, 0.0845, 0.1377];
m = 4.34;
d = 0.315;
ctf = 8.004e-4;

% input helixian path
t = 0:0.02:10;
st = sin(pi*t);
ct = cos(pi*t);
figure, title('input path')
plot3(0.4*t,0.4*st,0.6*ct)
hold off;

%x desired
xd_t = [0.4*t,0.4*st,0.6*ct];
b1d_t = [ct,st, zeros(1,501)];

kx = 16;
kv = 5.6;
kr = 8.81;
k_omega = 2.54;

