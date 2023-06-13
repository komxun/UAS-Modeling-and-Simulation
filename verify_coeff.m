clc, clear, close all 
dat = load('UAV_Data.mat');
    dat.Tbl_ALPHAT = dat.Tbl_ALPHAT * pi/180;    % convert to rad
    m = dat.Mass;   % [kg]

%%
    % --------------------------States--------------------------------
    % Translational velocities in body coordinate
    u = 200;
    v = 0;
    w = 0;
    % Angular velocities in body coordinate
    p = 0;
    q = 20 *pi/180;
    r = 50 *pi/180;
    %---------------Control inputs in Body coordinate-----------------
    T = 0;     % [N] Thrust
    del_R = 0;    % Roll inputs (Aerolon input)
    del_P = 0;    % Pitch inputs (Elevator input)
    del_Y = 0;    % Yaw inputs (Rudder input)

    a = 343;    % SoS
    rho = 1.225; 

%% 
V = norm([u v w]);          % [m/s] UAV total Speed
alpha = atan2(w,u);          % [rad] angle of attack
beta  = asin(v/V);          % [rad] sideslip angles
phi_T = atan2(tan(beta),sin(alpha));    % [rad] Roll angle
alpha_T = acos(cos(alpha)*cos(beta));  % [rad] Total angle of attack

QBAR = 0.5*rho*V*V;

Mach = V/a;       % Mach number (speed of sound = 343 m/s)
S = dat.S;        % [m^2] Reference wing area 
b = dat.D;        % [m] Semispan (= UAV diameter?)
g = 9.81;         % [m/s^2] Gravity

% disp(['Mach = ' num2str(Mach)])
% disp(['alpha_T = ' num2str(alpha_T)])

% Coordinate Transformation Matrix
B2A = rot1(-phi_T);                            % Body-to-Aeroballistics

% Moment of Inertia
I_x = dat.I_xx;
I_y = dat.I_yy;
I_z = dat.I_zz;

% Other variables
del_eff = (abs(del_P) + abs(del_Y))/2;

% Conversion to Aeroballistics coordinate
pqrA = B2A * [p; q; r];
pA = pqrA(1);
qA = pqrA(2);
rA = pqrA(3);

delARPY = B2A *[del_R; del_P; del_Y];
delR_A = delARPY(1);
delP_A = delARPY(2);
delY_A = delARPY(3);

%% Aerodynamics Coefficients in aeroballistics coordinates (checked x1)

method = 'linear'; % 'spline', 'nearest'
method2 = 'spline';

CX_a = interp1(dat.Tbl_MACH, dat.Tbl_CX_0, Mach, method,'extrap') ...
     + interp1(dat.Tbl_MACH, dat.Tbl_CX_ALPHAT, Mach, method,'extrap')*alpha_T...
     + interp1(dat.Tbl_MACH, dat.Tbl_CX_DEL_EFF, Mach, method,'extrap')*(del_eff)^2;

CY_a = interp2(dat.Tbl_ALPHAT, dat.Tbl_MACH, dat.Tbl_CY_PHIT, alpha_T*180/pi, Mach, method2)*sin(4*phi_T)...
     + interp1(dat.Tbl_MACH, dat.Tbl_CY_DEL_Y, Mach, method,'extrap')*delY_A;

CZ_a = interp2(dat.Tbl_ALPHAT, dat.Tbl_MACH, dat.Tbl_CZ_0, alpha_T*180/pi, Mach, method2)...
     + interp2(dat.Tbl_ALPHAT, dat.Tbl_MACH, dat.Tbl_CZ_PHIT, alpha_T*180/pi, Mach, method2)*(sin(2*phi_T))^2 ...
     + interp1(dat.Tbl_MACH, dat.Tbl_CZ_DEL_P, Mach, method,'extrap')*delP_A;

CL_a = interp1(dat.Tbl_MACH, dat.Tbl_CL_ALPHAT, Mach, method,'extrap')*(alpha_T^2)*sin(4*phi_T) ...
     + interp1(dat.Tbl_MACH, dat.Tbl_CL_P, Mach, method,'extrap')*(dat.D/(2*V))*pA ...
     + interp1(dat.Tbl_MACH, dat.Tbl_CL_DEL_R, Mach, method,'extrap')*delR_A;

CM_a = interp2(dat.Tbl_ALPHAT, dat.Tbl_MACH, dat.Tbl_CM_0, alpha_T*180/pi, Mach, method2) ...
     + interp2(dat.Tbl_ALPHAT, dat.Tbl_MACH, dat.Tbl_CM_PHIT, alpha_T*180/pi, Mach, method2)*(sin(2*phi_T))^2 ...
     + interp1(dat.Tbl_MACH, dat.Tbl_CM_Q, Mach, method,'extrap')*(dat.D/(2*V))*qA ...
     + interp1(dat.Tbl_MACH, dat.Tbl_CM_DEL_P, Mach, method,'extrap')*delP_A ...
     - CZ_a*(dat.XCG - dat.XREF)/dat.D;

CN_a = interp2(dat.Tbl_ALPHAT, dat.Tbl_MACH, dat.Tbl_CN_PHIT, alpha_T*180/pi, Mach, method2)*sin(4*phi_T)...
     + interp1(dat.Tbl_MACH, dat.Tbl_CN_R, Mach, method,'extrap')*(dat.D/(2*V))*rA ...
     + interp1(dat.Tbl_MACH, dat.Tbl_CN_DEL_Y, Mach, method,'extrap')*delY_A ...
     + CY_a*(dat.XCG - dat.XREF)/dat.D;

%% Aerodynamics Coefficients in body coordinate
CXYZ = B2A.' * [CX_a; CY_a; CZ_a];
CX = CXYZ(1)
CY = CXYZ(2)
CZ = CXYZ(3)

CLMN = B2A.' * [CL_a; CM_a; CN_a];
CL = CLMN(1)
CM = CLMN(2)
CN = CLMN(3)

%% Aerodynamic force/ moment model (F_A, M_A)

F_A = QBAR * S * [CX; CY; CZ];
M_A = QBAR * S * [b*CL; b*CM; b*CN];

%% The propulsion model (force / moment) (F_T, M_T)
F_T = [T; 0; 0]; 
M_T = [0; 0; 0];

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

