function at = sgn_target_accel(t, target_mode, p)
    %TARGETACCELMF_CODEGEN Code-generation-safe target acceleration.
    %
    % Inputs:
    %   t           time [s]
    %   target_mode 0 straight, 1 switching, 2 sinusoidal evasive test
    %   p           5x1 vector:
    %               [T1_SWITCH; T2_SWITCH; AT_PHASE1; AT_PHASE2; AT_PHASE3]
    %
    %#codegen
    
    T1 = p(1);
    T2 = p(2);
    A1 = p(3);
    A2 = p(4);
    A3 = p(5);
    
    if target_mode == 0
        at = 0.0;
    
    elseif target_mode == 1
        if t <= T1
            at = A1;
        elseif t <= T2
            at = A2;
        else
            at = A3;
        end
    
    else
        at = 0.25*sin(2*pi*0.20*t);
    end

end
