function [W,b] = xavier_init(in_size, out_size)
    %we want uniform random #'s from -bound to +bound
    bound = sqrt(6)/(sqrt(in_size + out_size));
    %get uniform random #'s from -1 to + 1
    unif_minus1_plus1 = 2*rand(out_size,in_size) - 1;
    %scale by bound
    W = bound * unif_minus1_plus1;
    %initialize b to zeros
    b = zeros(out_size,1);
    

end