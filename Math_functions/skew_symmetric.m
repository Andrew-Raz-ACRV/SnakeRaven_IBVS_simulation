%SKEW Create skew-symmetric matrix
function S = skew_symmetric(v)
    if length(v)==3
        % SO(3) case
        S = [  0   -v(3)  v(2)
              v(3)  0    -v(1)
             -v(2) v(1)   0];
    elseif length(v)==2
        % SO(2) case
        S = [0 -v; v 0];
    else
        error('argument must be a 1- or 3-vector');
    end
