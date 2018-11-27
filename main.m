
%just testing the walking robot for a fixed amount of time with the
%original sinusoidal controller thingy

% gam = 0;
% a = 0;
% tau = 3.84;
% k = -0.08;
% T = 100
% 
% 
% controller = @(t,y) original_controller(y,t,a,tau,k)
% 
% total_dist = simulate_walker(T,controller)
% 
% 
% 
% function F = original_controller(y,t,a,tau,k)
% 
%     
%     F = a*sin(2*pi/tau*t)+k*y(3);
% end

n_hidden = 64
W1_shape = [n_hidden,4]
n_W1 = W1_shape(1)*W1_shape(2)
b1_shape = [n_hidden,1]
n_b1 = b1_shape(1)
W2_shape = [1, n_hidden]
n_W2 = W2_shape(1)*W2_shape(2)
b2_shape = [1,1]
n_b2 = b2_shape(1)

n_params = n_W1 + n_b1 + n_W2 + n_b2
% 
% MU = zeros(n_params)
% SIGMA = eye(n_params)
% [W1,b1,W2,b2] = sample_params(MU,SIGMA,n_hidden)
% 
tanh_activ = @(x) tanh(x)
sigmoid_activ = @(x) 1./(1 + exp(-x));
linear_activ = @(x) x
% 
% y = [1;1;1;1]
% 
% F = nn_controller(y,W1,b1,W2,b2,sigmoid_activ,linear_activ)

%training params
sigma = .01
N = 1000
num_iters = 100
T = 100
max_steps = 100
initial_alpha = .00001
k = 0.1
params = zeros(1,n_params)

% sigma = .001
% N = 100
% num_iters = 100
% T = 100
% max_steps = 100
% alpha = .000001

%net params
activ1 = sigmoid_activ
activ2 = tanh_activ%sigmoid_activ%linear_activ
% activ2 = sigmoid_activ

mean_scores = zeros(1,num_iters);
for iter = 1:num_iters
    alpha = initial_alpha * exp(-k*iter)
    perturbations = mvnrnd(zeros(n_params,1),eye(n_params),N);  %might want to replace this with sampling from a univariate guassian since it's always spherical
    test_params = params + sigma*perturbations;
    scores = zeros(1,N);
    for i = 1:N
        scores(i) = score_params(test_params(i,:),n_hidden,activ1,activ2,T,max_steps,false);
%         i
%         scores(i)
    end
    disp('mean score: ')
    disp(mean(scores))
    mean_scores(i) = mean(scores);
    disp('iter: ')
    disp(iter)
    delta = alpha*(1/(N*sigma))*sum(scores'.*perturbations,1);
    disp('norm delta: ')
    disp(norm(delta))
    params = params + delta;
%     disp('norm params')
%     disp(norm(params))
    
    
end
plot(mean_scores)
score_params(params,n_hidden,activ1,activ2,T,max_steps,true)


    
        
    
       
        
    
