% This script initializes simulation data for the truePNG.mdl scheme
%
% Corso SGN - UNIPI - AA 2007/2008
% Author: Lorenzo Pollini (lpollini@dsea.unipi.it)

%target initialization
XT_INI = 1;
YT_INI = -1;
BETA_INI = (90 * pi/180);  % Inclinazione target, in radianti
VT = 0.25;
AT = 0;     % Accelerazione target

%drone initialization
XM_INI = -1;
YM_INI = -1;
GAMMA_INI = 0*(30*pi/180); %rad
VM = 0.15;
MAX_ACC = 0.5;
GUIDE_SEL = 1; %TRUE (GUIDE_SEL=1), PURE (GUIDE_SEL=0) 


%%%% DO NOT CHANGE BELOW THIS LINE

%compute other initial values
VXM_INI = VM * cos(GAMMA_INI);
VYM_INI = VM * sin(GAMMA_INI);
LOS_INI = atan2(YT_INI-YM_INI , XT_INI-XM_INI );
%tolerances
RTOL = 0.01;

% COSTANTE DI TEMPO FCS
Tautopilot = 0.35;
