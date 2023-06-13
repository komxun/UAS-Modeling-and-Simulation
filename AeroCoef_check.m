function [CX, CY, CZ, CL, CM, CN] = AeroCoef_check(V, Mach,phi_T, alpha_T, delR, delP, delY, p, q, r)

dat = load('UAV_Data.mat');
dat.Tbl_ALPHAT = dat.Tbl_ALPHAT;    % convert to rad
% alpha_T = alpha_T *180/pi;            % convert to degree

del_eff = (abs(delP) + abs(delY))/2;


B2A = rot1(-phi_T);                            % Body-to-Aeroballistics

delARPY = B2A *[delR; delP; delY];
delR_A = delARPY(1);
delP_A = delARPY(2);
delY_A = delARPY(3);

pqrA = B2A * [p; q; r];
pA = pqrA(1);
qA = pqrA(2);
rA = pqrA(3);

method = 'linear'; % 'spline', 'nearest'
method2 = 'spline';

CX_a = interp1(dat.Tbl_MACH, dat.Tbl_CX_0, Mach, method,'extrap') + interp1(dat.Tbl_MACH, dat.Tbl_CX_ALPHAT, Mach, method,'extrap')*alpha_T...
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
CX = CXYZ(1);
CY = CXYZ(2);
CZ = CXYZ(3);

CLMN = B2A.' * [CL_a; CM_a; CN_a];
CL = CLMN(1);
CM = CLMN(2);
CN = CLMN(3);

function out = rot1(x)
    out = [1 0 0; 0 cos(x) sin(x); 0 -sin(x) cos(x)];
end

end