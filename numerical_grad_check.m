close all
clear all




%define params of neural network
tanh_activ = @(x) tanh(x)
sigmoid_activ = @(x) 1./(1 + exp(-x));
linear_activ = @(x) x

activ1 = sigmoid_activ
activ2 = tanh_activ%sigmoid_activ%linear_activ

N = 100;
n_hidden = 100
T = 100;



[W1,b1] = xavier_init(4, n_hidden);
[W2,b2] = xavier_init(n_hidden, 1);
n_W1 = size(W1,1)*size(W1,2);
n_b1 = size(b1,1);
n_W2 = size(W2,1)*size(W2,2);
n_b2 = size(b2,1);

W1_flat = reshape(W1,[1,n_W1]);
b1_flat = reshape(b1,[1,n_b1]);
W2_flat = reshape(W2,[1,n_W2]);
b2_flat = reshape(b2,[1,n_b2]);
params = [1, W1_flat, b1_flat, W2_flat, b2_flat];
n_params = length(params)


param_grads = zeros(size(params));
score = score_params(params,n_hidden,activ1,activ2,T,false)
dparam = 10^-7;
for i = 1:length(params)
    params_pert = params;
    params_pert(i) = params_pert(i) + dparam;

    score_pert = score_params(params_pert,n_hidden,activ1,activ2,T,false);

    dscore = score_pert - score;

    param_grads(i) = dscore/dparam;

end


sigma = 10^-1; %variance in perturbatins from params
perturbations = mvnrnd(zeros(n_params,1),eye(n_params),N);  %might want to replace this with sampling from a univariate guassian since it's always spherical
%form population of N individuals with perturbed parameters, stored as
%a matrix
test_params = params + sigma*perturbations;
%score each individual in pop
scores = zeros(1,N);
for i = 1:N
    scores(i) = score_params(test_params(i,:),n_hidden,activ1,activ2,T,false);

end
disp('score params:')
disp(score_params(params,n_hidden,activ1,activ2,T,false))


disp('mean score: ')
disp(mean(scores))
%add mean score to mean scores
mean_scores(i) = mean(scores);

%calculate update to parameters
param_grads_stoch = (1/(N*sigma))*sum((scores - score)'.*perturbations,1);
% 
figure; hold on;
plot(param_grads)
plot(param_grads_stoch)
    

    