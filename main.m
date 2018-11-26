
%just testing the walking robot for a fixed amount of time with the
%original sinusoidal controller thingy

gam = 0;
a = 0;
tau = 3.84;
k = -0.08;
T = 100


controller = @(t,y) original_controller(y,t,a,tau,k)

total_dist = simulate_walker(T,controller)



function F = original_controller(y,t,a,tau,k)

    
    F = a*sin(2*pi/tau*t)+k*y(3);
end