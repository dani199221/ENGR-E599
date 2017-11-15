% iterate the loop with time intervals. equalent to the ROS loop
for t=0:0.02:10
    % define the constants for the system kx, kv, m, g, e1, e2, e3
    

    
    
    % inputs for the system
    % input 1: write the parametric equations for x, y, z and create the x_desired. 
    % input 2 :  b1_desired
    st = sin(pi*t);
    ct = cos(pi*t);
    xd_t = [0.4*t,0.4*st,0.6*ct];
    b1d_t = [ct,st, 0];




end