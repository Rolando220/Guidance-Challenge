function ang = sgn_wrap_pi(ang)
%SGN_WRAP_PI Wrap angle to (-pi, pi].
while ang > pi
    ang = ang - 2*pi;
end
while ang <= -pi
    ang = ang + 2*pi;
end
end
