clear all
close all


%define params of neural network
tanh_activ = @(x) tanh(x)
sigmoid_activ = @(x) 1./(1 + exp(-x));
linear_activ = @(x) x

activ1 = sigmoid_activ
activ2 = linear_activ%tanh_activ

n_hidden = 50;
[W1,b1] = xavier_init(5, n_hidden);
[W2,b2] = xavier_init(n_hidden, 1);
n_W1 = size(W1,1)*size(W1,2);
n_b1 = size(b1,1);
n_W2 = size(W2,1)*size(W2,2);
n_b2 = size(b2,1);

W1_flat = reshape(W1,[1,n_W1]);
b1_flat = reshape(b1,[1,n_b1]);
W2_flat = reshape(W2,[1,n_W2]);
b2_flat = reshape(b2,[1,n_b2]);
params = [W1_flat, b1_flat, W2_flat, b2_flat];
n_params = length(params)

%training params
sigma = 10^-1; %variance in perturbatins from params
N = 100 %populatin size
num_iters = 1000;  %number of updates we want to perform on params
T = 50 %largest time we're willing to run the sim for
weight = .1 %weight we assign to nn control 
% ===============benchmark PD controller==============

% this is usually around 67 for |epsilon| < .03 

kp = 16;
% kd = sqrt(abs(kp));
kd = 4;
k = [kp, kd];

s = 0.4;
alph = asin(0.5*s);
gam = 0;
a = 0;
tau = 3.84;
pd_controller = @(t,y) original_controller(y,t,a,tau,k, alph);
zero_controller = @(t,y) 0;

% benchmark_trials = 1000;
% pd_scores = zeros(1,benchmark_trials);
% zero_scores = zeros(1,benchmark_trials);
% parfor i = 1:benchmark_trials
%     [total_dist, total_step] = simulate_walker_terrain_stoc(T,pd_controller,false);
%     pd_scores(i) = total_dist;
%     [total_dist, total_step] = simulate_walker_terrain_stoc(T,zero_controller,false);
%     zero_scores(i) = total_dist;
% end

% disp('mean pd scores:')
% disp(mean(pd_scores))
% disp('mean zero scores:')
% disp(mean(zero_scores))

%% ====================================================



% mean scores will be used to keep track of the mean score across the
% populatin for each iter
figure(1)
figure(2)
baselines = zeros(1,num_iters);
norm_deltas = zeros(1,num_iters);
param_mat = zeros(length(params),num_iters);
m = 0; v = 0;
for iter = 1:num_iters
    param_mat(:,iter) = params;
    %get learning rate for this iter
%     alpha = initial_alpha * exp(-decay*(iter-1))
    %get perturbations on parameters
    perturbations = mvnrnd(zeros(n_params,1),eye(n_params),N);  %might want to replace this with sampling from a univariate guassian since it's always spherical
    %form population of N individuals with perturbed parameters, stored as
    %a matrix
    test_params = params + sigma*perturbations;
    %score each individual in pop
    baseline_scores = zeros(1,N);
    [W1,b1,W2,b2] = params_to_weights(params,n_hidden);
    hybrid_controller = @(t,y) pd_controller(t,y) + weight*nn_controller(y,W1,b1,W2,b2,activ1,activ2)
    parfor i = 1:N
    
        baseline_scores(i) = simulate_walker_terrain_stoc(T,hybrid_controller,false);

    end
    baseline = mean(baseline_scores)
    baselines(iter) = baseline;
    figure(1)
    plot(baselines(1:iter))
    title('1')
%     ylim([min([0,baselines]),max(baselines)+10])
    drawnow
    scores = zeros(1,N);
    parfor i = 1:N
        [W1,b1,W2,b2] = params_to_weights(test_params(i,:),n_hidden);
        hybrid_controller = @(t,y) pd_controller(t,y) + weight*nn_controller(y,W1,b1,W2,b2,activ1,activ2);
        scores(i) = simulate_walker_terrain_stoc(T,hybrid_controller,false);
    end
    
    %below code block is purely for evaluation
%     if mod(iter,10) == 1
%         eval_scores = zeros(1,1000);
%         parfor i = 1:1000
%             eval_scores(i) = score_params(params,n_hidden,activ1,activ2,T,false);
%         end
%         eval_mean = mean(eval_scores)
%         disp('eval_mean: ')
%         disp(eval_mean)
%     end
        
    

%     
%     hist(scores)
    disp('baseline score: ')
    disp(baseline)
    disp('iter: ')
    disp(iter)
    %calculate update to parameters
    grad = (1/(N*sigma))*sum((scores)'.*perturbations,1);
    [params, m, v] = adam_update(grad, params, m, v, iter);
    
%     delta = cap(delta, max_delta);
%     disp('norm delta: ')
%     disp(norm(delta))
%     norm_deltas(iter) = norm(delta);
%     figure(2)
%     plot(norm_deltas(1:iter))
%     drawnow
    
%     update parameters
%     params = params + delta;
%     disp(params(1))
    

end

        
function capped = cap(x,max_norm)

    if norm(x) > max_norm
        disp('capping')
        capped = max_norm * x/norm(x);
    else
        capped = x;
    end
end

function F = original_controller(y,t,a,tau,k, alpha)
%     F = a*sin(2*pi/tau*t)+ k(1)*y(3) + k(2)*y(4);
%     F = k(1)*y(3); 
    F = 0;
    
    if y(1)< 0
          F = k(1)*(-2*alpha - y(3)) + k(2)*(0 - y(4));
    end
end

function [params_tp1, m_tp1, v_tp1] = adam_update(grad, params_t, m_t, v_t, t)
    alpha = 5*10^-3;
    beta1 = .9;
    beta2 = .999;
    epsilon = 10^-8;
    
    m_tp1 = beta1*m_t + (1-beta1)*(-grad);  %throw a minus sign in front cuz we want to do gradient ascent
    v_tp1 = beta2*v_t + (1-beta2)*grad.^2;
    m_hat = m_tp1/(1-beta2^t);
    v_hat = v_tp1/(1 - beta2^t);
    params_tp1 = params_t - alpha*m_hat/(sqrt(v_hat) + epsilon);

end
    
