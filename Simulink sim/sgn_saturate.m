function y = sgn_saturate(u, lim)
%SGN_SATURATE Symmetric scalar saturation.
if u > lim
    y = lim;
elseif u < -lim
    y = -lim;
else
    y = u;
end
end
