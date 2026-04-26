% SetupForTrimAndLin_Aircraft6dof.m
%% Simulation Parameters
startTime = 0;
stopTime  = 60;
stepTime  = 0.01;

RatioToRealTime = 0.8;

%% Universal Constants
deg2rad = pi/180;

%% Control Allocation Matrices
MixMatrix   = [-1 1 0 0;
                1 1 0 0;
                0 0 1 0;
                0 0 0 1];
DeMixMatrix = inv(MixMatrix);

%% Actuator Initial Conditions and Parameters
dER_0 = 0; dEL_0 = 0; dE_0 = 0; dR_0 = 0;
tauER = 0.1; tauEL = 0.1; tauE = 0.1; tauR = 0.1;

%% Aircraft Mass and Geometry
%    mass(g)  xSize  ySize  zSize  xLoc   yLoc   zLoc
componentMassesAndGeom = ...
    [90     0.1    0.48   0.01   -0.23   0.44   0;      % Wing01+Servo
     90     0.1    0.48   0.01   -0.23  -0.44   0;      % Wing02+Servo
     13     0.075  0.35   0.002  -0.76   0     -0.16;   % Hor. Stab.
      0     0.08   0.002  0.18   -0.76   0     -0.09;   % Ver. Stab.
     72     0.065  0.035  0.015  -0.05   0      0.03;   % Battery
    106     0.87   0.07   0.07   -0.4    0      0;      % Fuselage
     27     0.05   0.03   0.005  -0.05   0      0.02;   % Motor Controller
     10     0.04   0.02   0.005   0.1    0      0.02;   % Radio
     20     0.05   0.01   0.01   -0.014  0      0;      % 2 Servos
     40     0.03   0.02   0.02    0.02   0      0.01;   % Motor
     12     0      0.26   0.025   0.05   0      0.01];  % Propeller

% Convert mass column to kg
componentMassesAndGeom(:,1) = componentMassesAndGeom(:,1) / 1000;

m = sum(componentMassesAndGeom(:,1));

% Center of mass
componentMoments(:,1) = componentMassesAndGeom(:,1) .* componentMassesAndGeom(:,5);
componentMoments(:,2) = componentMassesAndGeom(:,1) .* componentMassesAndGeom(:,6);
componentMoments(:,3) = componentMassesAndGeom(:,1) .* componentMassesAndGeom(:,7);
x_cm = sum(componentMoments) / m;
x_cm = x_cm';

% Component inertias (box formula)
for iComp = 1:size(componentMassesAndGeom,1)
    if componentMassesAndGeom(iComp,2) ~= 0
        componentInertias(iComp,1) = (1/12)*componentMassesAndGeom(iComp,1)*(componentMassesAndGeom(iComp,3)^2 + componentMassesAndGeom(iComp,4)^2);
        componentInertias(iComp,2) = (1/12)*componentMassesAndGeom(iComp,1)*(componentMassesAndGeom(iComp,4)^2 + componentMassesAndGeom(iComp,2)^2);
        componentInertias(iComp,3) = (1/12)*componentMassesAndGeom(iComp,1)*(componentMassesAndGeom(iComp,2)^2 + componentMassesAndGeom(iComp,3)^2);
    else
        componentInertias(iComp,1) = (1/12)*componentMassesAndGeom(iComp,1)*(componentMassesAndGeom(iComp,3)^2 + componentMassesAndGeom(iComp,4)^2);
        componentInertias(iComp,2) = 0;
        componentInertias(iComp,3) = 0;
    end
end

% Inertia tensor (parallel axis theorem)
J = zeros(3,3);
J(1,1) = sum(componentInertias(:,1)) + sum(componentMassesAndGeom(:,1) .* ((componentMassesAndGeom(:,6)-x_cm(2)).^2 + (componentMassesAndGeom(:,7)-x_cm(3)).^2));
J(2,2) = sum(componentInertias(:,2)) + sum(componentMassesAndGeom(:,1) .* ((componentMassesAndGeom(:,7)-x_cm(3)).^2 + (componentMassesAndGeom(:,5)-x_cm(1)).^2));
J(3,3) = sum(componentInertias(:,3)) + sum(componentMassesAndGeom(:,1) .* ((componentMassesAndGeom(:,5)-x_cm(1)).^2 + (componentMassesAndGeom(:,6)-x_cm(2)).^2));

% Off-diagonal terms: Ixy = Iyx, Ixz = Izx, Iyz = Izy
J(1,2) = -sum(componentMassesAndGeom(:,1) .* (componentMassesAndGeom(:,5)-x_cm(1)) .* (componentMassesAndGeom(:,6)-x_cm(2)));
J(1,3) = -sum(componentMassesAndGeom(:,1) .* (componentMassesAndGeom(:,7)-x_cm(3)) .* (componentMassesAndGeom(:,5)-x_cm(1)));
J(2,3) = -sum(componentMassesAndGeom(:,1) .* (componentMassesAndGeom(:,6)-x_cm(2)) .* (componentMassesAndGeom(:,7)-x_cm(3)));
J(2,1) = J(1,2);
J(3,1) = J(1,3);
J(3,2) = J(2,3);

%% Aerodynamic Surface Properties
x_s2 = [componentMassesAndGeom(3,5)+(1/4)*componentMassesAndGeom(3,2); componentMassesAndGeom(3,6); componentMassesAndGeom(3,7)];
x_s3 = [componentMassesAndGeom(4,5)+(1/4)*componentMassesAndGeom(4,2); componentMassesAndGeom(4,6); componentMassesAndGeom(4,7)];
x_s4 = [componentMassesAndGeom(1,5)+(1/4)*componentMassesAndGeom(1,2); componentMassesAndGeom(1,6); componentMassesAndGeom(1,7)];
x_s5 = [componentMassesAndGeom(2,5)+(1/4)*componentMassesAndGeom(2,2); componentMassesAndGeom(2,6); componentMassesAndGeom(2,7)];

n_s2 = [0; 0; -1];
n_s3 = [0; 1;  0];
n_s4 = [0; sin(0*pi/180); -cos(0*pi/180)];
n_s5 = [0; sin(0*pi/180); -cos(0*pi/180)];

c_s2 = componentMassesAndGeom(3,2);
c_s3 = componentMassesAndGeom(4,2);
c_s4 = componentMassesAndGeom(1,2);
c_s5 = componentMassesAndGeom(2,2);

c_u_s2 = 0.2*c_s2; c_u_s3 = 0.2*c_s3; c_u_s4 = 0.2*c_s4; c_u_s5 = 0.2*c_s5;

b_s2 = componentMassesAndGeom(3,3);
b_s3 = componentMassesAndGeom(4,4); % vertical stabilizer span (height), col 4 = zSize
b_s4 = componentMassesAndGeom(1,3);
b_s5 = componentMassesAndGeom(2,3);

S_s2 = c_s2*b_s2; S_s3 = c_s3*b_s3; S_s4 = c_s4*b_s4; S_s5 = c_s5*b_s5;
S_u_s2 = c_u_s2*b_s2; S_u_s3 = c_u_s3*b_s3; S_u_s4 = c_u_s4*b_s4; S_u_s5 = c_u_s5*b_s5;

AR_s2 = b_s2/c_s2;
AR_s3 = b_s3/c_s3;
AR_s4 = (b_s4+b_s5)/c_s4;
AR_s5 = (b_s4+b_s5)/c_s5;

CL0_s2 = 0; CL0_s3 = 0; CL0_s4 = 0.05; CL0_s5 = 0.05;
e_s2 = 0.8; e_s3 = 0.8; e_s4 = 0.9; e_s5 = 0.9;
i_s2 = -0.1; i_s3 = 0; i_s4 = 0.0; i_s5 = 0.0;
CD0_s2 = 0.01; CD0_s3 = 0.01; CD0_s4 = 0.01; CD0_s5 = 0.01;
CDa_s2 = 1; CDa_s3 = 1; CDa_s4 = 1; CDa_s5 = 1;
a0_s2 = 0; a0_s3 = 0; a0_s4 = 0.05; a0_s5 = 0.05;
CM0_s2 = 0; CM0_s3 = 0; CM0_s4 = -0.05; CM0_s5 = -0.05;
CMa_s2 = 0; CMa_s3 = 0; CMa_s4 = 0; CMa_s5 = 0;
dCL_du = 3;

%% Propulsion
% OLD WORKING REPLACED FOR HMWK 9 PROBLEM 2
%{
R_p1 = componentMassesAndGeom(11,3)/2;
n_p1 = [1; 0; 0];
n_int(:,1) = n_p1;
x_p1 = componentMassesAndGeom(11,5:7)';

ARvecUIUCp1 = [0;    0.4;  1.0;  1.6  ];
CTvecUIUCp1 = [0.1;  0.09; 0;   -0.09 ];
CPvecUIUCp1 = [0.05; 0.05; 0;   -0.05 ];

ARvecP1 = ARvecUIUCp1 * 30/pi * 2;
CTvecP1 = CTvecUIUCp1 * 2*(1/(2*pi))^2*(2^4);
CPvecP1 = CPvecUIUCp1 * 2*(1/(2*pi))^3*(2^5);
CQvecP1 = CPvecP1;

J_int(1)     = 0.01;
omDot_motor  = 0;
om_motor_init = 0;
%} 
%% Equivalent internal rotating assembly for 6 eVTOL props
% It represents the combined spinning inertia of 6 propellers.

R_p1 = componentMassesAndGeom(11,3)/2;
x_p1 = componentMassesAndGeom(11,5:7)';

Nprops_eq   = 6;
Jprop_each  = 1.5e-4;   % kg*m^2 per prop, choose a modest value
n_p1   = [1; 0; 0];
n_int(:,1)    = n_p1;

ARvecUIUCp1 = [0;    0.4;  1.0;  1.6  ];
CTvecUIUCp1 = [0.1;  0.09; 0;   -0.09 ];
CPvecUIUCp1 = [0.05; 0.05; 0;   -0.05 ];

ARvecP1 = ARvecUIUCp1 * 30/pi * 2;
CTvecP1 = CTvecUIUCp1 * 2*(1/(2*pi))^2*(2^4);
CPvecP1 = CPvecUIUCp1 * 2*(1/(2*pi))^3*(2^5);
CQvecP1 = CPvecP1;

% Equivalent single rotating assembly
J_int(1)     = Nprops_eq * Jprop_each;

% Keep trim and linearization easy:
% zero rotor acceleration and zero stored momentum at the operating point
omDot_motor   = 0;
om_motor_init = 0;

%% Motor Parameters
Rmotor   = 0.4;
Io_motor = 1.6;
Ki_motor = 0.009549;

%% Battery Parameters
nCellsSeries = 6;
VcellMax     = 4.2;

%% Earth / Atmosphere
Rearth  = 6371000;
g_E_N   = [0; 0; 9.81];
v_N_AE  = [0; 0; 0];

%% Initial Conditions
GoogleProvideLatLonAltOfUsBankInLa   = [34.051122; -118.254399; 310];
GuessAndCheckToGetPlaneOnUsBankInLa  = [34.05105;  -118.25439;  415];
FlightGearOffset = GuessAndCheckToGetPlaneOnUsBankInLa - GoogleProvideLatLonAltOfUsBankInLa;

InitPos_deg = GoogleProvideLatLonAltOfUsBankInLa + FlightGearOffset;
InitPos     = InitPos_deg .* [deg2rad; deg2rad; 1];

x_E_E_BE0 = (Rearth + InitPos(3)) * [cos(InitPos(1))*cos(InitPos(2));
                                       cos(InitPos(1))*sin(InitPos(2));
                                       sin(InitPos(1))];

InitAtt_deg = [0; 0; 0];
InitAtt     = InitAtt_deg * deg2rad;

InitQuat(1) = cos(InitAtt(1)/2)*cos(InitAtt(2)/2)*cos(InitAtt(3)/2) - sin(InitAtt(1)/2)*sin(InitAtt(2)/2)*sin(InitAtt(3)/2);
InitQuat(2) = sin(InitAtt(1)/2)*cos(InitAtt(2)/2)*cos(InitAtt(3)/2) - cos(InitAtt(1)/2)*sin(InitAtt(2)/2)*sin(InitAtt(3)/2);
InitQuat(3) = cos(InitAtt(1)/2)*sin(InitAtt(2)/2)*cos(InitAtt(3)/2) - sin(InitAtt(1)/2)*cos(InitAtt(2)/2)*sin(InitAtt(3)/2);
InitQuat(4) = cos(InitAtt(1)/2)*cos(InitAtt(2)/2)*sin(InitAtt(3)/2) - sin(InitAtt(1)/2)*sin(InitAtt(2)/2)*cos(InitAtt(3)/2);

InitVel   = [15; 0; 0];
InitRates = [0; 0; 0];

Fext_N = [0; 0; 0];
Mext_B = [0; 0; 0];