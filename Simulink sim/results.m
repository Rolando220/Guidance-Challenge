% This script plots simulation results for the truePNG.mdl scheme
%
% Corso SGN - UNIPI - AA 2007/2008
% Author: Lorenzo Pollini (lpollini@dsea.unipi.it)

close all

%trajectories
figure; 
plot(TARGET.signals.values(:,1),TARGET.signals.values(:,2),'r');
hold on; title('Trajectories'); grid on;
plot(TARGET.signals.values(1,1), TARGET.signals.values(1,2), 'x');
xlabel('X (m)'); ylabel('Y (m)');
plot(MISSILE.signals.values(:,1),MISSILE.signals.values(:,2),'b');
hold off;


%trajectories with LOS and ACC
figure; 
%number of LOS points:
NLOS=20;
MT = max(MISSILE.time);
DT = MT/NLOS;
LOSSAMPLES = [];
for t=0.1:DT:MT,
    LOSSAMPLES = [LOSSAMPLES; find(MISSILE.time>t,1) ];
end;
%plot trajectories
plot(TARGET.signals.values(:,1),TARGET.signals.values(:,2),'r');
hold on; title('Trajectories'); grid on;
xlabel('X (m)'); ylabel('Y (m)');
plot(MISSILE.signals.values(:,1),MISSILE.signals.values(:,2),'b');
%plot LOS lines
for t=1:length(LOSSAMPLES),
    plot([MISSILE.signals.values(LOSSAMPLES(t),1)  TARGET.signals.values(LOSSAMPLES(t),1)],[MISSILE.signals.values(LOSSAMPLES(t),2)  TARGET.signals.values(LOSSAMPLES(t),2)],'k');
end;
%plot ACC vectors
for t=1:length(LOSSAMPLES),
    if (GUIDE_SEL==1)
        quiver(MISSILE.signals.values(LOSSAMPLES(t),1), MISSILE.signals.values(LOSSAMPLES(t),2),...
        10*ACC.signals.values(LOSSAMPLES(t),1)*cos(pi/2+ENG.signals.values(LOSSAMPLES(t),2)),...
        10*ACC.signals.values(LOSSAMPLES(t),1)*sin(pi/2+ENG.signals.values(LOSSAMPLES(t),2)),'m');
    elseif (GUIDE_SEL==0)
        quiver(MISSILE.signals.values(LOSSAMPLES(t),1), MISSILE.signals.values(LOSSAMPLES(t),2),...
        10*ACC.signals.values(LOSSAMPLES(t),1)*cos(pi/2+MISSILE.signals.values(LOSSAMPLES(t),5)),...
        10*ACC.signals.values(LOSSAMPLES(t),1)*sin(pi/2+MISSILE.signals.values(LOSSAMPLES(t),5)),'m');
    end;
end;
axis equal;
hold off;

%missile acceleration 
figure;
plot(ACC.time,ACC.signals.values(:,1),'r',ACC.time,ACC.signals.values(:,2),'b');
xlabel('time (sec)'); ylabel('Acceleration (m/sec^2)');legend('a_m','a_c');
grid on; 

%range
figure;
plot(ENG.time,ENG.signals.values(:,1));
xlabel('time (sec)'); ylabel('Range (m)');
grid on;

%closing velocity
figure;
plot(ENG.time,-ENG.signals.values(:,4));
xlabel('time (sec)'); ylabel('Closing velocity (m/sec)');
grid on;

%los
figure;
plot(ENG.time,ENG.signals.values(:,2));
xlabel('time (sec)'); ylabel('LOS angle (rad)');
grid on;

%los rate
figure;
plot(ENG.time,ENG.signals.values(:,3));
xlabel('time (sec)'); ylabel('LOS rate (rad/sec)');
grid on;

%V missile
figure;
VM_res= sqrt(MISSILE.signals.values(:,3).^2 + MISSILE.signals.values(:,4).^2);
plot(MISSILE.time,VM_res);
xlabel('time (sec)'); ylabel('Missile velocity (m/sec)');
grid on;


