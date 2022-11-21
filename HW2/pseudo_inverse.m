function [p] = pseudo_inverse(X, c)
p = inv(transpose(X)*X)*transpose(X)*c;
end

