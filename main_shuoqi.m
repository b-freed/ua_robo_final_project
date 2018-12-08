clear all
close all
format short
clc

%% Environemnt 
% just testing the walking robot for a fixed amount of time with the
% original sinusoidal controller thingy

gam = 0;
a = 0;
tau = 3.84;

% kp = -0.08;
% kd = -sqrt(abs(kp/100));

kp = 16;
kd = sqrt(abs(kp));

k = [kp, kd];
T = 50;

%% test

y3_store = [];
y4_store = [];
F_store = [];
val_store = [];

%% 

s = 0.4;
alpha = asin(0.5*s);

controller = @(t,y) original_controller(y,t,a,tau,k, alpha);



% [total_dist, total_step] = simulate_walker(T,controller,ifplot)
[total_dist, total_step] = simulate_walker_terrain_stoc(T,controller,true)



%% 

function F = original_controller(y,t,a,tau,k, alpha)
%     F = a*sin(2*pi/tau*t)+ k(1)*y(3) + k(2)*y(4);
%     F = k(1)*y(3); 
    F = 0;
    
    if y(1)< 0
          F = k(1)*(-2*alpha - y(3)) + k(2)*(0 - y(4));
    end
end



%% NN
% 
% n_hidden = 64
% W1_shape = [n_hidden,4]
% n_W1 = W1_shape(1)*W1_shape(2)
% b1_shape = [n_hidden,1]
% n_b1 = b1_shape(1)
% W2_shape = [1, n_hidden]
% n_W2 = W2_shape(1)*W2_shape(2)
% b2_shape = [1,1]
% n_b2 = b2_shape(1)
% 
% n_params = n_W1 + n_b1 + n_W2 + n_b2
% % 
% % MU = zeros(n_params)
% % SIGMA = eye(n_params)
% % [W1,b1,W2,b2] = sample_params(MU,SIGMA,n_hidden)
% % 
% sigmoid_activ = @(x) 1./(1 + exp(-x));
% linear_activ = @(x) x
% % 
% % y = [1;1;1;1]
% % 
% % F = nn_controller(y,W1,b1,W2,b2,sigmoid_activ,linear_activ)
% 
% sigma = 1
% N = 100
% num_iters = 100
% T = 100
% alpha = .001
% params = zeros(1,n_params)
% sigma = 1
% activ1 = sigmoid_activ
% activ2 = linear_activ
% 
% 
% for iter = 1:num_iters
%     perturbations = mvnrnd(zeros(n_params,1),eye(n_params),N);  %might want to replace this with sampling from a univariate guassian since it's always spherical
%     test_params = params + sigma*perturbations;
%     scores = zeros(1,N)
%     for i = 1:N
%         scores(i) = score_params(test_params(i,:),n_hidden,activ1,activ2,T,false);
%         i
%         scores(i)
%     end
%     disp('mean score: ')
%     mean(scores)
%     disp('iter: ')
%     disp(iter)
%     delta = alpha*(1/(N*sigma))*sum(scores*perturbations)
%     params = params + delta;
% end
