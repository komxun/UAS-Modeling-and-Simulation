clc, clear, close all

Init_Pos     =       [ 0.0, 0.0, -1000 ]' ;  % [m] Xe, Ye, Ze       
Init_Vel     =       [ 238, 0, 0]'; % [m/s]
Init_Euler   =       [ 0.0, 0.0, 0.0 ]';   % [rad]  
Init_Rate    =       [ 0.0, 0.0, 0.0 ]';   % [rad/s?]

ALT = Init_Pos(3);
X0 = [Init_Vel; Init_Rate; Init_Euler; Init_Pos];
U0 = [0 2*pi/180 0 0];  % [Thrust Aerolon Elevator Rudder]

tspan = 1:0.01:10;
% options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);
[time, State] = ode45(@(t,x) Missile_EoM(t, x, U0, ALT), tspan, X0);

% State = [u v w p q r phi theta psi]


%% Plot
figure
subplot(3,1,1)
plot(time, State(:,1), 'LineWidth', 1.5)
title('U')
subplot(3,1,2)
plot(time, State(:,2), 'LineWidth', 1.5)
title('V')
ylim([-0.5,0.5])
subplot(3,1,3)
plot(time, State(:,3), 'LineWidth', 1.5)
title('W')
ylim([-0.5,0.5])

figure
subplot(3,1,1)
plot(time, State(:,4)*180/pi, 'LineWidth', 1.5)
title('P')
ylim([-0.5,0.5])
subplot(3,1,2)
plot(time, State(:,5)*180/pi, 'LineWidth', 1.5)
title('Q')
subplot(3,1,3)
plot(time, State(:,6)*180/pi, 'LineWidth', 1.5)
title('R')
ylim([-0.5,0.5])

figure
subplot(3,1,1)
plot(time, State(:,7)*180/pi, 'LineWidth', 1.5)
title('\Phi')
ylim([-0.5,0.5])
subplot(3,1,2)
plot(time, State(:,8)*180/pi, 'LineWidth', 1.5)
title('\Theta')
subplot(3,1,3)
plot(time, State(:,9)*180/pi, 'LineWidth', 1.5)
title('\Psi')
ylim([-0.5,0.5])


figure
subplot(3,1,1)
plot(time, State(:,10), 'LineWidth', 1.5)
title('X_e')
subplot(3,1,2)
plot(time, State(:,11), 'LineWidth', 1.5)
title('Y_e')
ylim([-0.5,0.5])
subplot(3,1,3)
plot(time, -State(:,12), 'LineWidth', 1.5)
title('Z_e')

