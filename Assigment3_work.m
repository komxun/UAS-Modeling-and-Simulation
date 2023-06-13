clc, clear, close all

dat = load('UAV_Data.mat');

%% -----------Users inputs---------------------
% Mach number index (1 ~ 6) [ 0.2    0.3    0.4    0.5    0.6    0.7 ]
M_idx = 1;

% Alpha_T index (1 ~ 5) [ 0     4     8    12    16 ] 
AT_idx = 1;

ALT = 0;    % [m] Altitude

%% Unknown Parameters

% Translational velocities in body coordinate
u0 = ;
v0 = ;
w0 = ;

% Angular velocities in body coordinate
p0 = ;       
q0 = ;
r0 = ;

% Euler Angles
phi0 = ;
psi0 = ;
theta0 = ;

X0 = 
tspan  =
[time, State] = ode45(@(t,x) UAV_EoM(t, state, m, F_A, F_T, F_G, M_A, M_T, I_x, I_y, I_z), tspan, X0);

% Thrust parameters
C_T = ;           % Thrust coefficient
delta_T = ;       % Engine throttle

%% 
alpha = arctan(w/u);        % angle of attack
beta  = arctan(v/V);        % sideslip angles
phi_T = arctan(tan(beta)/sin(alpha));    % Roll angle

V = norm([u v w]);          % [m/s] UAV total Speed

% Constant
[AMACH, QBAR, H_p, APR] = adc(V, ALT);  % Atmospheric Data Calculation
S = dat.S;        % [m^2] Reference wing area 
b = dat.D;        % [m] Semispan (= UAV diameter?)
nu = ;            % Engine propeller efficiency
g = 9.81;         % [m/s^2] Gravity

q_bar = 0.5 * rho * V^2;                    % Dynamic pressure

M = dat.Tbl_MACH(M_idx)                    % Mach number
alpha_T = dat.Tbl_ALPHAT(AT_idx) *pi/180   % [rad] Total Angle-of-Attack 
a = V/M;                                   % [m/s] Speed of sound

% Coordinate Transformation Matrix
B2E = rot3(-psi) * rot2(-theta) * rot1(-phi);  % Body-to-Earth
B2A = rot1(-phi_T);                            % Body-to-Aeroballistics

% Control inputs in Body coordinate
del_R = ;    % Roll inputs
del_P = ;    % Pitch inputs
del_Y = ;    % Yaw inputs

% Moment of Inertia
I_x = dat.I_xx;
I_y = dat.I_yy;
I_z = dat.I_zz;

% Other variables
del_eff = (abs(del_P) + abs(del_Y))/2;

% Conversion to Aeroballistics coordinate
[pA; qA; rA] = B2A * [p; q; r];
[delA_R; delA_P; delA_Y] = B2A *[del_R; del_P; del_Y];

%% Aerodynamics Coefficients in aeroballistics coordinates (checked x1)

CaX = dat.Tbl_CX_0(M_idx) + dat.Tbl_CX_ALPHAT(M_idx)*alpha_T + dat.Tbl_CX_DEL_EFF(M_idx)*(del_eff)^2;

CaY = dat.Tbl_CY_PHIT(AT_idx, M_idx)*sin(4*phi_T) + dat.Tbl_CY_DEL_Y(M_idx)*delA_Y;

CaZ = dat.Tbl_CZ_0(AT_idx, M_idx) + dat.Tbl_CZ_PHIT(AT_idx, M_idx)*(sin(2*phi_T))^2 ...
    + dat.Tbl_CZ_DEL_P(M_idx)*delA_P;

CaL = dat.Tbl_CL_ALPHAT(M_idx)*(alpha_T^2)*sin(4*phi_T) + dat.Tbl_CL_P(M_idx)*(dat.D/(2*V))*pA ...
    + dat.Tbl_CL_DEL_R(M_idx)*delA_R;

CaM = dat.Tbl_CM_0(AT_idx, M_idx) + dat.Tbl_CM_PHIT(AT_idx, M_idx)*(sin(2*phi_T))^2 ...
    + dat.Tbl_CM_Q(M_idx)*(dat.D/(2*V))*qA + dat.Tbl_CM_DEL_P(M_idx)*delA_P ...
    - CaZ*(dat.XCG - dat.XREF)/dat.D;

CaN = dat.Tbl_CN_PHIT(AT_idx, M_idx)*sin(4*phi_T) + dat.Tbl_CN_R(M_idx)*(dat.D/(2*V))*rA ...
    + dat.Tbl_CN_DEL_Y(M_idx)*delA_Y + CaY*(dat.XCG - dat.XREF)/dat.D;

%% Aerodynamics Coefficients in body coordinate
[CX; CY; CZ] = B2A' * [CaX; CaY; CaZ];
[CL; CM; CN] = B2A' * [CaL; CaM; CaN];

%% Navigation Equation

function [V_xe, V_ye, V_ze] = Nav(u, v, w, B2E)
    [V_xe; V_ye; V_ze] = B2E*[u; v; w];
end

%% Aerodynamic force/ moment model (F_A, M_A)

T = nu * C_T * delta_T;     % Thrust
F_A = QBAR * S * [CX; CY; CZ];
M_A = QBAR * S * [b*CL; b*CM; b*CN];

%% The propulsion model (force / moment) (F_T, M_T)
T = ; % Thrust located at the missile tail
F_T = [T; 0; 0]; 
M_T = [0; 0; 0];

%% The environment model
F_G = dat.Mass*g*[-sin(theta); sin(phi)*cos(theta); cos(phi)*cos(theta)];

%% The actuator model

%% Appendix 1 - Rotational matrices
function out = rot3(x)
    out = [cos(x) sin(x) 0; -sin(x) cos(x) 0; 0 0 1;];
end

function out = rot2(x)
    out = [cos(x) 0 -sin(x); 0 1 0; sin(x) 0 cos(x)];
end

function out = rot1(x)
    out = [1 0 0; 0 cos(x) sin(x); 0 -sin(x) cos(x)];
end

% Appendix 2 -Admospheric Data Calculation
function ADC = adc(V, ALT)

    %ALT = 1500 ;
    
    R0   = 2.377e-03 ;                   % Sea level density
    TFAC = 1.0 - 0.703e-05*ALT ;         % Fahrenheit to Celsius Conversion
    T    = 519.0 * TFAC ;
    
    if ALT >= 35000.0
        T = 390.0;       
    end
    
    RHO   = R0 * (TFAC^4.14) ;
    AMACH = V/sqrt(1.4*1716.3*T) ;
    QBAR  = 0.5*RHO*V*V ;
    H_p   = 16545.146*(1-(0.001928*T)) ;              % Pressure Altitude
    APR   = (1. - 0.00000688*ALT)^(5.2561) ;	      % Ambient pressure ratio
    
    ADC = [AMACH, QBAR, H_p, APR, RHO] ;
end
