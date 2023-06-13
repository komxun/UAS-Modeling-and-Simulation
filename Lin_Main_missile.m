%/////////////////////////////////////////////////////////////////////////%

%.. Matlab Initialize 

    clc ;           close all ;             clear all ; 
    
%.. Load Sim Parameters

    Sim_Parameters ; 
    
%.. Load Trim Solution Found 

    load Trim_Solution_missile4.mat ;
    
%.. Linear Model Calculation

    %-------------------Please Complete This Part ------------------------%
    [ sizes, x0, names] =  missile_Lin_model;
    %---------------------------------------------------------------------%
    
    disp('///////////////////////////////////////////////')
    disp('          Check order of state variables')
    disp('///////////////////////////////////////////////')
    names{:}

    %-------------------Please Complete This Part ------------------------%
    %   - Name :                                                          %
    %   - X = [ ]'                                                        %
    %   - U = [ ]'                                                        %
    %---------------------------------------------------------------------%

    % Initial conditions (trim value)
    %-------------------Please Complete This Part ------------------------%
    X0(1) = 0;              % X
    X0(2) = 0;              % Y    
    X0(3) = -Alt_Trim;      % Z  
    X0(4) = x_trim(4);      % p
    X0(5) = x_trim(5);      % q
    X0(6) = x_trim(6);      % r 
    X0(7) = x_trim(7);      % phi
    X0(8) = x_trim(8);      % theta
    X0(9) = x_trim(9);      % psi
    X0(10) = x_trim(1);     % u
    X0(11) = x_trim(2);     % v
    X0(12) = x_trim(3);     % w  
    
    U0 = u_trim;       
    %---------------------------------------------------------------------%
    
    % Linearization
	%-------------------Please Complete This Part ------------------------%
    [A, B, C, D]    =   linmod('missile_Lin_model', X0, U0);   
    %---------------------------------------------------------------------%
    
    % Longitudinal Dynamics 
   	%/////////////////////////////////////////////////////////////////%
    %   - X = [ U, W, Q, THETA ]'                                     %
    %   - U = [ del_T, del_e ]'                                       %
    %/////////////////////////////////////////////////////////////////%
    
    A_long          =       zeros( 4, 4 ) ; 
    B_long          =       zeros( 4, 1 ) ; 
    C_long          =       zeros( 4, 4 ) ; 
    D_long          =       zeros( 4, 1 ) ; 
    
    %-------------------Please Complete This Part ------------------------%
    Idx_U = 10;
    Idx_W = 12;
    Idx_Q = 5;
    Idx_THETA  = 8;
    %---------------------------------------------------------------------%
    
    A_long(1,:)     =       [ A(    Idx_U,Idx_U), A(    Idx_U,Idx_W), A(    Idx_U,Idx_Q), A(    Idx_U,Idx_THETA) ] ;
    A_long(2,:)     =       [ A(    Idx_W,Idx_U), A(    Idx_W,Idx_W), A(    Idx_W,Idx_Q), A(    Idx_W,Idx_THETA) ] ; 
    A_long(3,:)     =       [ A(    Idx_Q,Idx_U), A(    Idx_Q,Idx_W), A(    Idx_Q,Idx_Q), A(    Idx_Q,Idx_THETA) ] ; 
    A_long(4,:)     =       [ A(Idx_THETA,Idx_U), A(Idx_THETA,Idx_W), A(Idx_THETA,Idx_Q), A(Idx_THETA,Idx_THETA) ] ;
    
    B_Thrust     =       [ B(Idx_U,1), B(Idx_W,1), B(Idx_Q,1), B(Idx_THETA,1) ]' ; 
    B_long     =       [ B(Idx_U,2), B(Idx_W,2), B(Idx_Q,2), B(Idx_THETA,2) ]' ; 
    
 	C_long(1,:)     =       [ C(    Idx_U,Idx_U), C(    Idx_U,Idx_W), C(    Idx_U,Idx_Q), C(    Idx_U,Idx_THETA) ] ;
    C_long(2,:)     =       [ C(    Idx_W,Idx_U), C(    Idx_W,Idx_W), C(    Idx_W,Idx_Q), C(    Idx_W,Idx_THETA) ] ; 
    C_long(3,:)     =       [ C(    Idx_Q,Idx_U), C(    Idx_Q,Idx_W), C(    Idx_Q,Idx_Q), C(    Idx_Q,Idx_THETA) ] ; 
    C_long(4,:)     =       [ C(Idx_THETA,Idx_U), C(Idx_THETA,Idx_W), C(Idx_THETA,Idx_Q), C(Idx_THETA,Idx_THETA) ] ;
    
%     D_long(:,1)     =       [ D(Idx_U,1), D(Idx_W,1), D(Idx_Q,1), D(Idx_THETA,1) ]' ; 
    D_long     =       [ D(Idx_U,2), D(Idx_W,2), D(Idx_Q,2), D(Idx_THETA,2) ]' ; 
    
    A_long
    B_long
    C_long
    D_long
    
    % Lateral Dynamics 
   	%/////////////////////////////////////////////////////////////////%
    %   - X = [ V, P, R, PHI ]'                                       %
    %   - U = [ del_a, del_r ]'                                       %
    %/////////////////////////////////////////////////////////////////%    
    
	A_lat           =       zeros( 4, 4 ) ; 
    B_lat           =       zeros( 4, 2 ) ; 
    C_lat           =       zeros( 4, 4 ) ; 
    D_lat           =       zeros( 4, 2 ) ; 
    
 	%-------------------Please Complete This Part ------------------------%
    Idx_V = 11;
    Idx_P = 4;
    Idx_R = 6;
    Idx_PHI  = 7;
    %---------------------------------------------------------------------%
    
    A_lat(1,:)      =       [ A(  Idx_V,Idx_V), A(  Idx_V,Idx_P), A(  Idx_V,Idx_R), A(  Idx_V,Idx_PHI) ] ;
    A_lat(2,:)      =       [ A(  Idx_P,Idx_V), A(  Idx_P,Idx_P), A(  Idx_P,Idx_R), A(  Idx_P,Idx_PHI) ] ; 
    A_lat(3,:)      =       [ A(  Idx_R,Idx_V), A(  Idx_R,Idx_P), A(  Idx_R,Idx_R), A(  Idx_R,Idx_PHI) ] ; 
    A_lat(4,:)      =       [ A(Idx_PHI,Idx_V), A(Idx_PHI,Idx_P), A(Idx_PHI,Idx_R), A(Idx_PHI,Idx_PHI) ] ;
    
    B_lat(:,1)      =       [ B(Idx_V,3), B( Idx_P,3), B(Idx_R,3), B(Idx_PHI,3) ]' ; 
    B_lat(:,2)      =       [ B(Idx_V,4), B( Idx_P,4), B(Idx_R,4), B(Idx_PHI,4) ]' ; 
    
    C_lat(1,:)      =       [ C(  Idx_V,Idx_V), C(  Idx_V,Idx_P), C(  Idx_V,Idx_R), C(  Idx_V,Idx_PHI) ] ;
    C_lat(2,:)      =       [ C(  Idx_P,Idx_V), C(  Idx_P,Idx_P), C(  Idx_P,Idx_R), C(  Idx_P,Idx_PHI) ] ; 
    C_lat(3,:)      =       [ C(  Idx_R,Idx_V), C(  Idx_R,Idx_P), C(  Idx_R,Idx_R), C(  Idx_R,Idx_PHI) ] ; 
    C_lat(4,:)      =       [ C(Idx_PHI,Idx_V), C(Idx_PHI,Idx_P), C(Idx_PHI,Idx_R), C(Idx_PHI,Idx_PHI) ] ;
    
    D_lat(:,1)      =       [ D(Idx_V,3), D( Idx_P,3), D(Idx_R,3), D(Idx_PHI,3) ]' ; 
    D_lat(:,2)      =       [ D(Idx_V,4), D( Idx_P,4), D(Idx_R,4), D(Idx_PHI,4) ]' ;     
    
    A_lat
    B_lat
    C_lat
    D_lat
    
  
    save( 'Llinear_Model.mat', 'A_long', 'B_long', 'C_long', 'D_long', 'A_lat', 'B_lat', 'C_lat', 'D_lat') ;
    
%% Stability Analysis
    eigenvalues_long=eig(A_long)
    long_sys = ss(A_long, B_long, C_long, D_long)
    figure(1);
    plot(eigenvalues_long, '*')
    title('Longitudinal Mode')
    xlabel('Real Axis') 
    ylabel('Imaginary Axis')
    xline(0), yline(0)
    grid on
    
    eigenvalues_lat_movement=eig(A_lat)
    lat_sys = ss(A_lat, B_lat, C_lat, D_lat)
    figure(2);
    plot(eigenvalues_lat_movement, '*')
    title('Lateral Mode')
    xlabel('Real Axis') 
    ylabel('Imaginary Axis')
    xline(0), yline(0)
    grid on