%/////////////////////////////////////////////////////////////////////////%
%                                                                         %
%   - Name : UAS_Trim_Main.m                                              %
%                                                                         %
%                               - Created by C. H. Lee, 15/10/2018        %
%                                                                         %
%/////////////////////////////////////////////////////////////////////////%

%.. Matlab Initialize 

    clc, close all ; clear all ; 
    
%.. Global Variables

    global      Alt_Trim            Speed_Trim
    global      Theta_dot_Trim      Turn_dot_Trim       
    global      G_Turn              Gamma_Trim
    global      Init_Cnstr1         Init_Cnstr2         Init_Cnstr3
    
%.. Load Sim Parameters

    Sim_Parameters; 
    %.. Trim Conditions Chosen 
cond = 1;
switch cond
    case 1
        Alt_Trim = 0; % [m] Operating Altitude  
        a = 340.268;
        Mach = 0.3;   % Operating Mach number
    case 2
        Alt_Trim = 0; % [m] Operating Altitude
        a = 340.268;
        Mach = 0.4;   % Operating Mach number
    case 3
        Alt_Trim = 0; % [m] Operating Altitude 
        a = 340.268;
        Mach = 0.6;   % Operating Mach number
    case 4
        Alt_Trim = 0; % [m] Operating Altitude
        a = 340.268;
        Mach = 0.7;   % Operating Mach number
    case 5
        Alt_Trim = 1000; % [m] Operating Altitude 
        a = 336.408;
        Mach = 0.7;   % Operating Mach number
    case 6
        Alt_Trim = 2000; % [m] Operating Altitude
        a = 332.504;
        Mach = 0.7;   % Operating Mach number
    case 7
        Alt_Trim = 3000; % [m] Operating Altitude
        a = 328.553;
        Mach = 0.7;   % Operating Mach number
    case 8
        Alt_Trim = 4000; % [m] Operating Altitude 
        a = 324.554;
        Mach = 0.7;   % Operating Mach number
end

Speed_Trim          =       Mach*a ;    % [m/s] Operating Speed

% Trim solutions
switch cond
    case 1
        X0 = [Speed_Trim 0 0 0 0 0 0 0 0 0 0 0];
        U0 = [0 0 0 0];
    case 2
        X0 = [Speed_Trim 0 0 0 0 0 0 0 0 0 0 0];
        U0 = [0 0 0 0];
    case 3
        X0 = [Speed_Trim 0 0 0 0 0 0 0 0 0 0 0];
        U0 = [0 0 0 0];
    case 4
        X0 = [Speed_Trim 0 0 0 0 0 0 0 0 0 0 0];
        U0 = [0 0 0 0];
    case 5
        X0 = [Speed_Trim 0 0 0 0 0 0 0 0 0 0 0];
        U0 = [0 0 0 0];
    case 6
        X0 = [Speed_Trim 0 0 0 0 0 0 0 0 0 0 0];
        U0 = [0 0 0 0];
    case 7
        X0 = [Speed_Trim 0 0 0 0 0 0 0 0 0 0 0];
        U0 = [0 0 0 0];
    case 8
        X0 = [Speed_Trim 0 0 0 0 0 0 0 0 0 0 0];
        U0 = [0 0 0 0];
end


%.. Initial Values of Additional Constraints for Trim     

    Init_Cnstr1         =       0.0 ;   % Initial State for Constraint1 (Speed)
    Init_Cnstr2         =       0.0 ;   % Initial State for Constraint2 (Pull-up)
    Init_Cnstr3         =       0.0 ;   % Initial State for Constraint3 (Turn)
    
%.. Pull-up Constraint for Trim Calculation

    Theta_dot_Trim      =       0.0 * UNIT_DEG2RAD ;    % Default = 0.0 
    
%.. Coordinate Turn Constraint (CTC) for Trim Calculation

    Turn_dot_Trim       =       0.0 * UNIT_DEG2RAD ;    % Default = 0.0
    G_Turn              =       Turn_dot_Trim * Speed_Trim / UNIT_GRAV ;   
    
%.. Rate of Climb Constraint (ROC) for Trim Calculation

    Gamma_Trim          =       0.0 * UNIT_DEG2RAD ;    % Default = 0.0   


%.. Find names and ordering of states from Simulink model    
    
%     %-------------------Please Complete This Part ------------------------%
    [ sizes, x0, names] =  missile_trim_model;
%     %---------------------------------------------------------------------%
    
    disp('///////////////////////////////////////////////')
    disp('          Check order of state variables')
    disp('///////////////////////////////////////////////')
    names{:}

% %.. Initial Guess for Trim Conditions

    %-------------------Please Complete This Part ------------------------%
%     X0(1)           =   Speed_Trim;    % [m/s]  u                                 
%     X0(2)           =   0;             % [m/s]  v                              
%     X0(3)           =   0;             % [m/s]  w                             
%     X0(4)           =   0;             % [rad/s] P             
%     X0(5)           =   0;             % [rad/s] Q            
%     X0(6)           =   0;             % [rad/s] R        
%     X0(7)           =   0;             % [rad] Phi               
%     X0(8)           =   0;             % [rad] Theta            
%     X0(9)           =   0;             % [rad] Psi                  
%     X0(10)          =   0;             % CTC                            
%     X0(11)          =   0;             % ROC                             
%     X0(12)          =   0;             % Speed Constraint                            
%     
%     U0(1)           =    0;           % [HP]     Thrust                               
%     U0(2)           =    0;           % [rad]   delta_p               
%     U0(3)           =    0;           % [rad]   delta_r                 
%     U0(4)           =    0;           % [rad]   delta_y               
    %---------------------------------------------------------------------%
    
% %.. Trim Calculation 

    %-------------------Please Complete This Part ------------------------%
    [ x_trim, u_trim, y_trim, xd_trim ]  = trim('missile_trim_model', X0.', U0.');
    %---------------------------------------------------------------------%
    filename = ['Trim_Solution_missile' num2str(cond) '.mat'];
    save( filename, 'x_trim', 'u_trim', 'y_trim', 'xd_trim',...
        'Speed_Trim', 'Alt_Trim', 'Mach' ) ;
    xd_trim
%     
%     %-------------------Please Complete This Part ------------------------%
    disp('///////////////////////////////////////////////')
    disp('          Trim State and Trim Input            ')
    disp('///////////////////////////////////////////////')
    fprintf(' \n Altitude  = %3.4f m ', Alt_Trim) ;
    fprintf(' \n Mach      = %3.4f \n ', Mach) ;
    
    fprintf(' \n ') ;
    fprintf(' U      = %3.4f m/s\n ', y_trim(1)     ) ;
    fprintf(' V      = %3.4f m/s\n ', y_trim(2)     ) ;
    fprintf(' W      = %3.4f m/s\n ', y_trim(3)     ) ;
    fprintf(' P      = %3.4f deg/s\n ', y_trim(4)  * 180/pi ) ;
    fprintf(' Q      = %3.4f deg/s\n ', y_trim(5)  * 180/pi) ;
    fprintf(' R      = %3.4f deg/s\n ', y_trim(6)  * 180/pi) ;   
    fprintf(' PHI    = %3.4f deg\n ', y_trim(7)  * 180/pi ) ;
    fprintf(' THETA  = %3.4f deg\n ', y_trim(8)  * 180/pi ) ;
    fprintf(' PSI    = %3.4f deg\n ', y_trim(9)  * 180/pi ) ;      
    fprintf(' ALPHA  = %3.4f deg\n ', y_trim(10) * 180/pi    ) ;
    fprintf(' BETA   = %3.4f deg\n ', y_trim(11) * 180/pi     ) ;
    fprintf(' VT     = %3.4f m/s\n ', y_trim(12)     ) ;
    fprintf(' \n ') ;
    fprintf(' del_T  = %3.4f N\n ',    u_trim(1) ) ;
    fprintf(' del_R  = %3.4f deg\n ',   u_trim(2) * UNIT_RAD2DEG ) ;
    fprintf(' del_P  = %3.4f deg\n ',   u_trim(3) * UNIT_RAD2DEG ) ;
    fprintf(' del_Y  = %3.4f deg\n ',   u_trim(4) * UNIT_RAD2DEG ) ;
%      %---------------------------------------------------------------------%
