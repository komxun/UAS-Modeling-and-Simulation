clc, clear, close all

for j = 1:8
    filename = ['Trim_Solution_missile' num2str(j) '.mat'];
    dat = load(filename);
    Mach(1,j) = dat.Mach;
    Alt(1,j) = dat.Alt_Trim;
    Speed_Trim(1,j) = dat.Speed_Trim;
    Y_trim(:,j) = dat.y_trim;
    U_trim(:,j) = dat.u_trim;
end
Y_trim(4:11,:) = Y_trim(4:11,:)*180/pi;
U_trim(2:4,:) = U_trim(2:4,:)*180/pi;


format bank
Mat = num2cell([Mach; Alt; Y_trim;]);
head_row_name = {'Parameter','case 1', 'case 2','case 3','case 4','case 5','case 6','case 7', 'case 8'};
head_col_name = { 'Mach', 'Altitude', 'u_trim', 'v_trim', 'w_trim', 'P_trim', 'Q_trim',...
    'R_trim', 'Phi_trim', 'Theta_trim', 'Psi_trim', 'Alpha_trim', 'Beta_trim', 'V_trim'};
Mat = ([head_col_name' Mat]);

t = cell2table(Mat, 'VariableNames',head_row_name)

Mat2 = num2cell([Mach; Alt; U_trim;]);
head_row_name2 = {'Parameter','case 1', 'case 2','case 3','case 4','case 5','case 6','case 7', 'case 8'};
head_col_name2 = { 'Mach', 'Altitude', 'Thurst_trim', 'delR_trim', 'delP_trim', 'delA_trim'};
Mat2 = ([head_col_name2' Mat2]);

t2 = cell2table(Mat2, 'VariableNames',head_row_name)
