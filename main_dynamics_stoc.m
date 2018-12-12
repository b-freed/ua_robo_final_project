clear all
close all
format long

%!!!!!! Decide which controller to use here!!!!!

global controller_type % 1: P controller. 2: PD controller 3: hybrid controller 4: NN controller
controller_type = 1; 

global trial_number % decide how many trials we want average over
trial_number = 50;  

global add_terrain_stoc  % decide if you want to add in terrain stochasticity
add_terrain_stoc = false; 

%% ===============benchmark PD controller==============

T = 50; %largest time we're willing to run the sim for
n_hidden = 10;
% this is usually around 67 for |epsilon| < .03 

s = 0.4;
alph = asin(0.5*s);
gam = 0;
a = 0;
tau = 3.84;
weight = .1;

switch controller_type
    case 1 % switch to P controller
        p_controller = @(t,y) PD_controller(y,t,a,tau,alph);
        controller = p_controller;

    case 2 % switch to PD controller
        pd_controller = @(t,y) P_controller(y,t,a,tau,alph);
        controller = pd_controller;

    case 3 % switch to hybrid controller
        %define params of neural network
        pd_controller = @(t,y) PD_controller(y,t,a,tau,alph);
        
        tanh_activ = @(x) tanh(x)
        sigmoid_activ = @(x) 1./(1 + exp(-x));
        linear_activ = @(x) x

        activ1 = sigmoid_activ
        activ2 = linear_activ

        load hybrid_params_opt.mat  %load trained params
        [W1,b1,W2,b2] = params_to_weights(params_opt,n_hidden);
        hybrid_controller = @(t,y) pd_controller(t,y) + weight*nn_controller(y,W1,b1,W2,b2,activ1,activ2);
        controller = hybrid_controller;
        
    case 4
        disp('nn_controller not yet implemented!')
    
end

%% The main for loop

% itialize the varibles
total_dist = 0;
total_step = 0;

for i = 1:trial_number
     
    [dist, step] = simulate_walker_dynamic_stoc(T,controller,false);

    total_dist = total_dist + dist;
    total_step = total_step + step;
    
end

ave_dist = total_dist / trial_number;
ave_step = total_step / trial_number;


fprintf('average walking distance over %d trials is: %f \n', trial_number, ave_dist);
fprintf('average steps taken over %d trials is: %f \n', trial_number, ave_step);

%%
% save('copy_best_phi_dot_X_theta_dot_Y.mat', 'thetaX_phiY');

        
function capped = cap(x,max_norm)

    if norm(x) > max_norm
        disp('capping')
        capped = max_norm * x/norm(x);
    else
        capped = x;
    end
end

function F = PD_controller(y,t,a,tau,alpha)

    F = 0;

    if y(1)< 0
        kp = 16;
        kd = 4;
        k = [kp, kd];
        F = k(1)*(-2*alpha - y(3)) + k(2)*(0 - y(4));
    end
end


function F = P_controller(y,t,a,tau, alpha)

    k = -0.08;
    F = k(1)*y(3); 
    
end

function [params_tp1, m_tp1, v_tp1] = adam_update(grad, params_t, m_t, v_t, t)
    alpha = 5*10^-4;
    beta1 = .9;
    beta2 = .999;
    epsilon = 10^-8;
    
    m_tp1 = beta1*m_t + (1-beta1)*(-grad);  %throw a minus sign in front cuz we want to do gradient ascent
    v_tp1 = beta2*v_t + (1-beta2)*grad.^2;
    m_hat = m_tp1/(1-beta2^t);
    v_hat = v_tp1/(1 - beta2^t);
    params_tp1 = params_t - alpha*m_hat/(sqrt(v_hat) + epsilon);

end
    
