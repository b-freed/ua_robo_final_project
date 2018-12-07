function score = score_params(params,n_hidden,activ1,activ2,T,disp_plot)

    scale = 1;
    [W1,b1,W2,b2] = params_to_weights(params,n_hidden);
    controller = @(t,y) scale*nn_controller(y,W1,b1,W2,b2,activ1,activ2);
    total_dist = simulate_walker_terrain_stoc(T,controller,disp_plot);
    score = total_dist;



end