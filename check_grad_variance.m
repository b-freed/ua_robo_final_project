% This is a script to attempt to systematically determine the reason for
% the high-variance gradients

% Initializing neural network

clear all
% close all

N = 100; %this is "batch size"
T = 50;
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


%% effect of sigma on gradient variance
sigma_vec = [10,1,10^-1]%,10^-2,10^-3]
for i = 1:length(sigma_vec)
    sigma = sigma_vec(i)
    perturbations = mvnrnd(zeros(n_params,1),eye(n_params),N);
    test_params = params + sigma*perturbations;
    
    grad_mat = zeros(length(params),10);
    for n = 1:10
        scores = zeros(1,N);
        parfor j = 1:N
            scores(j) = score_params(test_params(j,:),n_hidden,activ1,activ2,T,false);
        end
        baseline_scores = zeros(1,N);
        parfor j = 1:N
            baseline_scores(j) = score_params(params,n_hidden,activ1,activ2,T,false);
        end
        baseline = mean(baseline_scores)
        grad = (1/(N*sigma))*sum((scores - baseline)'.*perturbations,1);
        grad_mat(:,n) = grad;
    end
    figure;plot(grad_mat);title('N='+string(N)+', sigma = '+string(sigma) +' w/ baseline');
    
    
    
end


        
        
    
        
        
    
    

