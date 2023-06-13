function [sdot] = Missile_EoM(t, state, U, ALT)
    
    sdot = zeros(9,1);
    dat = load('UAV_Data.mat');
    dat.Tbl_ALPHAT = dat.Tbl_ALPHAT * pi/180;    % convert to rad
    m = dat.Mass;   % [kg]

%%
    % --------------------------States--------------------------------
    % Translational velocities in body coordinate
    u = state(1);
    v = state(2);
    w = state(3);

    % Angular velocities in body coordinate
    p = state(4);
    q = state(5);
    r = state(6);

    % Euler Angles
    phi   = state(7);
    theta = state(8);
    psi   = state(9);

    %---------------Control inputs in Body coordinate-----------------
    T = U(1);     % [N] Thrust
    del_R = U(2);    % Roll inputs (Aerolon input)
    del_P = U(3);    % Pitch inputs (Elevator input)
    del_Y = U(4);    % Yaw inputs (Rudder input)

    % UAV is axis-symmetric
    I_xz = 0;

%% 
V = norm([u v w]);          % [m/s] UAV total Speed
alpha = atan2(w,u);          % [rad] angle of attack
beta  = asin(v/V);          % [rad] sideslip angles
phi_T = atan2(tan(beta),sin(alpha));    % [rad] Roll angle
alpha_T = acos(cos(alpha)*cos(beta));  % [rad] Total angle of attack

% Constant
[~, a, ~, rho] = atmosisa(ALT);
QBAR = 0.5*rho*V*V;

Mach = V/a;       % Mach number (speed of sound = 343 m/s)
S = dat.S;        % [m^2] Reference wing area 
b = dat.D;        % [m] Semispan (= UAV diameter?)
g = 9.81;         % [m/s^2] Gravity

% disp(['Mach = ' num2str(Mach)])
% disp(['alpha_T = ' num2str(alpha_T)])

% Coordinate Transformation Matrix
B2E = rot3(-psi) * rot2(-theta) * rot1(-phi);  % Body-to-Earth
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

method = 'spline'; 

CX_a = interp1(dat.Tbl_MACH, dat.Tbl_CX_0, Mach, method) + interp1(dat.Tbl_MACH, dat.Tbl_CX_ALPHAT, Mach, method)*alpha_T...
     + interp1(dat.Tbl_MACH, dat.Tbl_CX_DEL_EFF, Mach, method)*(del_eff)^2;

CY_a = interp2(dat.Tbl_ALPHAT, dat.Tbl_MACH, dat.Tbl_CY_PHIT, alpha_T, Mach, method)*sin(4*phi_T)...
     + interp1(dat.Tbl_MACH, dat.Tbl_CY_DEL_Y, Mach, method)*delY_A;

CZ_a = interp2(dat.Tbl_ALPHAT, dat.Tbl_MACH, dat.Tbl_CZ_0, alpha_T, Mach, method)...
     + interp2(dat.Tbl_ALPHAT, dat.Tbl_MACH, dat.Tbl_CZ_PHIT, alpha_T, Mach, method)*(sin(2*phi_T))^2 ...
     + interp1(dat.Tbl_MACH, dat.Tbl_CZ_DEL_P, Mach, method)*delP_A;

CL_a = interp1(dat.Tbl_MACH, dat.Tbl_CL_ALPHAT, Mach, method)*(alpha_T^2)*sin(4*phi_T) ...
     + interp1(dat.Tbl_MACH, dat.Tbl_CL_P, Mach, method)*(dat.D/(2*V))*pA ...
     + interp1(dat.Tbl_MACH, dat.Tbl_CL_DEL_R, Mach, method)*delR_A;

CM_a = interp2(dat.Tbl_ALPHAT, dat.Tbl_MACH, dat.Tbl_CM_0, alpha_T, Mach, method) ...
     + interp2(dat.Tbl_ALPHAT, dat.Tbl_MACH, dat.Tbl_CM_PHIT, alpha_T, Mach, method)*(sin(2*phi_T))^2 ...
     + interp1(dat.Tbl_MACH, dat.Tbl_CM_Q, Mach, method)*(dat.D/(2*V))*qA ...
     + interp1(dat.Tbl_MACH, dat.Tbl_CM_DEL_P, Mach, method)*delP_A ...
     - CZ_a*(dat.XCG - dat.XREF)/dat.D;

CN_a = interp2(dat.Tbl_ALPHAT, dat.Tbl_MACH, dat.Tbl_CN_PHIT, alpha_T, Mach, method)*sin(4*phi_T)...
     + interp1(dat.Tbl_MACH, dat.Tbl_CN_R, Mach, method)*(dat.D/(2*V))*rA ...
     + interp1(dat.Tbl_MACH, dat.Tbl_CN_DEL_Y, Mach, method)*delY_A ...
     + CY_a*(dat.XCG - dat.XREF)/dat.D;

%% Aerodynamics Coefficients in body coordinate
CXYZ = B2A.' * [CX_a; CY_a; CZ_a];
CX = CXYZ(1);
CY = CXYZ(2);
CZ = CXYZ(3);

CLMN = B2A.' * [CL_a; CM_a; CN_a];
CL = CLMN(1);
CM = CLMN(2);
CN = CLMN(3);

%% Navigation Equation
Vxyz_e = B2E*[u; v; w];
V_xe = Vxyz_e(1);
V_ye = Vxyz_e(2);
V_ze = Vxyz_e(3);

%% Aerodynamic force/ moment model (F_A, M_A)

F_A = QBAR * S * [CX; CY; CZ];
M_A = QBAR * S * [b*CL; b*CM; b*CN];

%% The propulsion model (force / moment) (F_T, M_T)
F_T = [T; 0; 0]; 
M_T = [0; 0; 0];

%% The environment model
F_G = dat.Mass*g*[-sin(theta); sin(phi)*cos(theta); cos(phi)*cos(theta)];


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

%% Output
    % Force Equations
    sdot(1) = r*v - q*w + F_A(1)/m + F_T(1)/m + F_G(1)/m;       % u dot
    sdot(2) = p*w - r*u + F_A(2)/m + F_T(2)/m + F_G(2)/m;       % v dot
    sdot(3) = q*u - p*v + F_A(3)/m + F_T(3)/m + F_G(3)/m;       % w dot

    % Moment Equations (axis-symmetric UAV: I_xz = 0)
    sdot(4) = M_A(1)/I_x + M_T(1)/I_x - (I_z - I_y)*q*r/I_x;    % p dot
    sdot(5) = M_A(2)/I_y + M_T(2)/I_y - (I_x - I_z)*p*r/I_y;    % q dot
    sdot(6) = M_A(3)/I_z + M_T(3)/I_z - (I_y - I_x)*p*q/I_z;    % r dot

    % Kinematic Equations
    sdot(7) = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);% phi dot
    sdot(8) = q*cos(phi) - r*sin(phi);                          % theta dot
    sdot(9) = q*sin(phi)*sec(theta) + r*cos(phi)*sec(theta);    % psi dot

    % Navigation
    sdot(10) = V_xe;
    sdot(11) = V_ye;
    sdot(12) = V_ze;
    sdot(isnan(sdot)) =0;

end