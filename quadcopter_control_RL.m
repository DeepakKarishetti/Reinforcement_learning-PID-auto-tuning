clear
clc

s = load('quadcopter_cube_diag.mat','data');
s.data(:,:)
% Time steps
time_steps = 0:0.1:2.5;

% Initialize states
S_t = zeros(12,1);
Q = zeros(length(time_steps), length(time_steps),12);
try
    % Initialize Q tables, kP and kD if files exist
    Q_file = load('Q_matrices.mat','Q');
    kP_file = load('kP_vectors.mat','kP');
    kD_file = load('kD_vectors.mat','kD');
    
    
    Q_1 = Q_file.Q(:,:,1);
    Q_2 = Q_file.Q(:,:,2);
    Q_3 = Q_file.Q(:,:,3);
    Q_4 = Q_file.Q(:,:,4);
    Q_5 = Q_file.Q(:,:,5);
    Q_6 = Q_file.Q(:,:,6);
    Q_7 = Q_file.Q(:,:,7);
    Q_8 = Q_file.Q(:,:,8);
    Q_9 = Q_file.Q(:,:,9);
    Q_10 = Q_file.Q(:,:,10);
    Q_11 = Q_file.Q(:,:,11);
    Q_12 = Q_file.Q(:,:,12);
    
    
    kPx_vec = kP_file.kP(:,1);
    kPy_vec = kP_file.kP(:,2);
    kPz_vec = kP_file.kP(:,3);
    kPphi_vec = kP_file.kP(:,4);
    kPtheta_vec = kP_file.kP(:,5);
    kPpsi_vec = kP_file.kP(:,6);

    kDx_vec = kD_file.kD(:,1);
    kDy_vec = kD_file.kD(:,2);
    kDz_vec = kD_file.kD(:,3);
    kDphi_vec = kD_file.kD(:,4);
    kDtheta_vec = kD_file.kD(:,5);
    kDpsi_vec = kD_file.kD(:,6);
catch
    % Initialize Q tables, kP and kD if files doesn't exist
    Q_1 = zeros(length(time_steps), length(time_steps));
    Q_2 = zeros(length(time_steps), length(time_steps));
    Q_3 = zeros(length(time_steps), length(time_steps));
    Q_4 = zeros(length(time_steps), length(time_steps));
    Q_5 = zeros(length(time_steps), length(time_steps));
    Q_6 = zeros(length(time_steps), length(time_steps));
    Q_7 = zeros(length(time_steps), length(time_steps));
    Q_8 = zeros(length(time_steps), length(time_steps));
    Q_9 = zeros(length(time_steps), length(time_steps));
    Q_10 = zeros(length(time_steps), length(time_steps));
    Q_11 = zeros(length(time_steps), length(time_steps));
    Q_12 = zeros(length(time_steps), length(time_steps));
    
    kPx_vec = 30*rand(length(time_steps),1);
    kPy_vec = 30*rand(length(time_steps),1);
    kPz_vec = 60*rand(length(time_steps),1);
    kPphi_vec = 20 + 100*rand(length(time_steps),1);
    kPtheta_vec = 20 + 100*rand(length(time_steps),1);
    kPpsi_vec = 30*rand(length(time_steps),1);

    kDx_vec = 40*rand(length(time_steps),1);
    kDy_vec = 40*rand(length(time_steps),1);
    kDz_vec = 60*rand(length(time_steps),1);
    kDphi_vec = 10 + 80*rand(length(time_steps),1);
    kDtheta_vec = 10 + 80*rand(length(time_steps),1);
    kDpsi_vec = 10*rand(length(time_steps),1);
end




e_p = zeros(6,length(time_steps));
e_d = zeros(6,length(time_steps));

% Learning rates
alpha_1 = 0.3;
alpha_2 = 0.3;

% Exploration rate
epsilon = 0.27;

% Episodes
max_episodes = 51;

%
X_min = zeros(3,1);
X_max = ones(3,1);
%
omega_min = -0.3*ones(3,1);
omega_max = 0.3*ones(3,1);
%
X_dot_min = zeros(3,1);
X_dot_max = ones(3,1);
%
omega_dot_min = -1.5* ones(3,1);
omega_dot_max = 1.5 * ones(3,1);
%
X_con = zeros(3,1);
X_dot_con = zeros(3,1);
omega_con = zeros(3,1);
omega_dot_con = zeros(3,1);
%
% Number of buckets each variable is defined into
N = length(time_steps);
%
% Discretization
n_x = 0;
n_y = 0;
n_z = 0;
n_x_dot = 0;
n_y_dot = 0;
n_z_dot = 0;
n_phi = 0;
n_theta = 0;
n_psi = 0;
n_phi_dot = 0;
n_theta_dot = 0;
n_psi_dot = 0;
%
% Choosing actions based on greedy method
action = zeros(12,26);

% Action p
action_pos_p = zeros(12,length(time_steps));
action_alt_p = zeros(12,length(time_steps));

% Action d
action_pos_d = zeros(12,length(time_steps));
action_alt_d = zeros(12,length(time_steps));

% Output from controller
u_p = zeros(3,length(time_steps));
u_d = zeros(3,length(time_steps));

% Threshold
pos_thresh = 0.1;
alt_thresh = 0.1;

% 
S_pos = zeros(6,length(time_steps));
S_alt = zeros(6,length(time_steps));

e_p_lim = 0.1;
e_d_lim = 0.1;

df = 0.995;

% Assumptions
mass = 0.25; % mass of quadcopter
iX = 0.03; % kgm^2
iY = 0.03; % kgm^2
iZ = 0.03; % kgm^2

thrustFactor = 0.1;
dragFactor = 0.1;
armLength = 0.1;


kP = zeros(length(time_steps),6);
kD = zeros(length(time_steps),6);

episode = 1;
while episode < max_episodes
    episode % To indicate the current episode state for the user
    x_vec_next = zeros(6,1);
    omega_vec_next = zeros(6,1);
    % Decay
    if episode < 0.6*max_episodes
        epsilon = 1 / (1 + exp(episode)) + 0.001;
    else
        epsilon = 0;
    end
    t = 0;
    for time = 0:0.1:2.4
        t = t+1;
        %
        %
        % Discritization of S_t states
        % Position X
        if x_vec_next(1) < X_min(1)
            n_x = 1;
        elseif x_vec_next(1) > X_max(1)
            n_x = 26;
        else
            n_x = floor(abs(((x_vec_next(1)/(X_max(1)-X_min(1)))*N)))+1;
        end
        % Position Y
        if x_vec_next(2) < X_min(2)
            n_y = 1;
        elseif x_vec_next(2) > X_max(2)
            n_y = 26;
        else
            n_y = floor(abs(((x_vec_next(2)/(X_max(2)-X_min(2)))*N)))+1;
        end
        % Position Z
        if x_vec_next(3) < X_min(3)
            n_z = 1;
        elseif x_vec_next(3) > X_max(3)
            n_z = 26;
        else
            n_z = floor(abs(((x_vec_next(3)/(X_max(3)-X_min(3)))*N)))+1;
        end
        % Velocity along X
        if x_vec_next(4) < X_dot_min(1)
            n_x_dot = 1;
        elseif x_vec_next(4) > X_dot_max(1)
            n_x_dot = 26;
        else
            n_x_dot = floor(abs(((x_vec_next(4)/(X_dot_max(1)-X_dot_min(1)))*N)))+1;
        end
        % Velocity along Y
        if x_vec_next(5) < X_dot_min(2)
            n_y_dot = 1;
        elseif x_vec_next(5) > X_dot_max(2)
            n_y_dot = 26;
        else
            n_y_dot = floor(abs(((x_vec_next(5)/(X_dot_max(2)-X_dot_min(2)))*N)))+1;
        end
        % Velocity along Z
        if x_vec_next(6) < X_dot_min(3)
            n_z_dot = 1;
        elseif x_vec_next(6) > X_dot_max(3)
            n_z_dot = 26;
        else
            n_z_dot = floor(abs(((x_vec_next(6)/(X_dot_max(3)-X_dot_min(3)))*N)))+1;
        end
        
        % Orientation Phi
        if omega_vec_next(1) < omega_min(1)
            n_phi = 1;
        elseif omega_vec_next(1) > omega_max(1)
            n_phi = 26;
        else
            n_phi = floor(abs(((omega_vec_next(1)/(omega_max(1)-omega_min(1)))*N)))+1;
        end
        % Orientation Theta
        if omega_vec_next(2) < omega_min(2)
            n_theta = 1;
        elseif omega_vec_next(2) > omega_max(2)
            n_theta = 26;
        else
            n_theta = floor(abs(((omega_vec_next(2)/(omega_max(2)-omega_min(2)))*N)))+1;
        end
        % Orientation Psi
        if omega_vec_next(3) < omega_min(3)
            n_psi = 1;
        elseif omega_vec_next(3) > omega_max(3)
            n_psi = 26;
        else
            n_psi = floor(abs(((omega_vec_next(3)/(omega_max(3)-omega_min(3)))*N)))+1;
        end
        % Angular velocity about X axis
        if omega_vec_next(4) < omega_dot_min(1)
            n_phi_dot = 1;
        elseif omega_vec_next(4) > omega_dot_max(1)
            n_phi_dot = 26;
        else
            n_phi_dot = floor(abs(((omega_vec_next(4)/(omega_dot_max(1)-omega_dot_min(1)))*N)))+1;
        end
        % Angular velocity about Y axis
        if omega_vec_next(5) < omega_dot_min(2)
            n_theta_dot = 1;
        elseif omega_vec_next(5) > omega_dot_max(2)
            n_theta_dot = 26;
        else
            n_theta_dot = floor(abs(((omega_vec_next(5)/(omega_dot_max(2)-omega_dot_min(2)))*N)))+1;
        end
        % Angular velocity about Z axis
        if omega_vec_next(6) < omega_dot_min(3)
            n_psi_dot = 1;
        elseif omega_vec_next(6) > omega_dot_max(3)
            n_psi_dot = 26;
        else
            n_psi_dot = floor(abs(((omega_vec_next(6)/(omega_dot_max(3)-omega_dot_min(3)))*N)))+1;
        end
        %
        %
        eta = rand(1,1); % Random choice of eta
        
        
        % Actions for each State
        
        if eta < epsilon
            ind1 = randi([1 26],1,1);
            action_x = kPx_vec(ind1);
        else
            [~,ind1] = max(Q_1(n_x,:));
            action_x = kPx_vec(ind1);
        end
        
        if eta < epsilon
            ind2 = randi([1 26],1,1);
            action_y = kPy_vec(ind2);
        else
            [~,ind2] = max(Q_2(n_y,:));
            action_y = kPy_vec(ind2);
        end
        
        if eta < epsilon
            ind3 = randi([1 26],1,1);
            action_z = kPz_vec(ind3);
        else
            [~,ind3] = max(Q_3(n_z,:));
            action_z = kPz_vec(ind3);
        end
        
        if eta < epsilon
            ind4 = randi([1 26],1,1);
            action_phi = kPphi_vec(ind4);
        else
            [~,ind4] = max(Q_4(n_phi,:));
            action_phi = kPphi_vec(ind4);
        end
        
        if eta < epsilon
            ind5 = randi([1 26],1,1);
            action_theta = kPtheta_vec(ind5);
        else
            [~,ind5] = max(Q_5(n_theta,:));
            action_theta = kPtheta_vec(ind5);
        end
        
        if eta < epsilon
            ind6 = randi([1 26],1,1);
            action_psi = kPpsi_vec(ind6);
        else
            [~,ind6] = max(Q_6(n_psi,:));
            action_psi = kPpsi_vec(ind6);
        end
        
        if eta < epsilon
            ind7 = randi([1 26],1,1);
            action_xdot = kDx_vec(ind7);
        else
            [~,ind7] = max(Q_7(n_x_dot,:));
            action_xdot = kDx_vec(ind7);
        end
        
        if eta < epsilon
            ind8 = randi([1 26],1,1);
            action_ydot = kDy_vec(ind8);
        else
            [~,ind8] = max(Q_8(n_y_dot,:));
            action_ydot = kDy_vec(ind8);
        end
        
        if eta < epsilon
            ind9 = randi([1 26],1,1);
            action_zdot = kDz_vec(ind9);
        else
            [~,ind9] = max(Q_9(n_z_dot,:));
            action_zdot = kDz_vec(ind9);
        end
        
        if eta < epsilon
            ind10 = randi([1 26],1,1);
            action_phidot = kDphi_vec(ind10);
        else
            [~,ind10] = max(Q_10(n_phi_dot,:));
            action_phidot = kDphi_vec(ind10);
        end
        
        if eta < epsilon
            ind11 = randi([1 26],1,1);
            action_thetadot = kDtheta_vec(ind11);
        else
            [~,ind11] = max(Q_11(n_theta_dot,:));
            action_thetadot = kDtheta_vec(ind11);
        end
        
        if eta < epsilon
            ind12 = randi([1 26],1,1);
            action_psidot = kDpsi_vec(ind12);
        else
            [~,ind12] = max(Q_12(n_psi_dot,:));
            action_psidot = kDpsi_vec(ind12);
        end
        
        
        % Choosing gains using respective Action for each state
        kPx = kPx_vec(ind1);
        kPy = kPy_vec(ind2);
        kPz = kPz_vec(ind3);
        kPphi = kPphi_vec(ind4);
        kPtheta = kPtheta_vec(ind5);
        kPpsi = kPpsi_vec(ind6);
        
        kDx = kDx_vec(ind7);
        kDy = kDy_vec(ind8);
        kDz = kDz_vec(ind9);
        kDphi = kDphi_vec(ind10);
        kDtheta = kDtheta_vec(ind11);
        kDpsi = kDpsi_vec(ind12);
        
        
        % Error Calculation for each state
        %
        % u_p = k_p(e) + k_d(e_dot)     
        % e = x_d - x_c   e_dot = x_dot_d -x_dot_c
        %
        % u_d = k_p(e) + k_d(e_dot)
        % e = phi_d - phi_c     e_dot = phi_dot_d - phi_dot_c
        %
        x_ref = zeros(6,1);
        x_ref(1:3) = s.data(t,1:3);
        x_ref(4:6) = s.data(t,7:9);
        e_p(:,t) = x_ref - S_pos(:,t);
        
        omega_ref = zeros(6,1);
        omega_ref(1:3) = s.data(t,4:6);
        omega_ref(4:6) = s.data(t,10:12);
        e_d(:,t) = omega_ref - S_alt(:,t);
        
        
        x_err = e_p(1,t);
        y_err = e_p(2,t);
        z_err = e_p(3,t);
        x_dot_err = e_d(1,t);
        y_dot_err = e_d(2,t);
        z_dot_err = e_d(3,t);
        
        phi_err = e_p(4,t);
        theta_err = e_p(5,t);
        psi_err = e_p(6,t);
        phi_dot_err = e_d(4,t);
        theta_dot_err = e_d(5,t);
        psi_dot_err = e_d(6,t);
        
        
        % Thrust and Torque calculations
        z_ddot = kPz*z_err + kDz*z_dot_err + (0.25*9.81);
        x_ddot = kPx*x_err + kDx*x_dot_err;
        y_ddot = kPy*y_err + kDy*y_dot_err;
        
        del_u1 = z_ddot*0.9; % Determines the Thrust along z-axis with 90% for abnormal conditions
        psiDes = 0.0;        % yaw rotation is fixed
        
        R = [sin(psiDes) cos(psiDes);-cos(psiDes) sin(psiDes)];
        invR = (0.25/del_u1)*inv(R);
        
        phi_des = invR(1,:)*[x_ddot;y_ddot];
        theta_des = invR(2,:)*[x_ddot;y_ddot];
        
        delPhi = (kPphi*phi_err + kDphi*phi_dot_err)*0.9;           % Torque about X-axis
        delTheta = (kPtheta*theta_err + kDtheta*theta_dot_err)*0.9; % Torque about Y-axis
        delPsi = (kPpsi*psi_err + kDpsi*psi_dot_err)*0.9;           % Torque about Z-axis
        
        % Intial states for simulation from current to next state
        x0 = x_vec_next(1);
        y0 = x_vec_next(2);
        z0 = x_vec_next(3);
        xDot0 = x_vec_next(4);
        yDot0 = x_vec_next(5);
        zDot0 = x_vec_next(6);
        
        phi0 = omega_vec_next(1);
        theta0 = omega_vec_next(2);
        psi0 = omega_vec_next(3);
        phiDot0 = omega_vec_next(4);
        thetaDot0 = omega_vec_next(5);
        psiDot0 = omega_vec_next(6);
        
        
        % Simulation from current time to the next time step
        simout = sim('quadcopter_control_training_2018a','StartTime',num2str(0),'StopTime',...
            num2str(0.1),'OutputOption','SpecifiedOutputTimes','OutputTimes',num2str(0.1));
        
        % Observing new states
        x_vec_next(1:3,1) = (simout.simout.Data(end,1:3))';
        x_vec_next(4:6,1) = (simout.simout.Data(end,7:9))';
        omega_vec_next(1:3,1) = (simout.simout.Data(end,4:6))';
        omega_vec_next(4:6,1) = (simout.simout.Data(end,10:12))';
        state_lim = 0.075;
        
        % Assigning Rewards based on the next state observed
        % Reward of 100 was given for exact match of states
        % Reward of 10 was given for states lying within a tolerance limit
        % Reward of -2 was given for states away from the limit
        if (x_vec_next(1)>(s.data(t+1,1)-state_lim)) && (x_vec_next(1)<(s.data(t+1,1)+state_lim))
            R_px = 10;
            R_dx = 10;
        elseif (x_vec_next(1) == s.data(t+1,1))
            R_px = 100;
            R_dx = 100;
        else
            R_px = -2;
            R_dx = -2;
        end
        
        if (x_vec_next(2)>(s.data(t+1,2)-state_lim)) && (x_vec_next(2)<(s.data(t+1,2)+state_lim))
            R_py = 10;
            R_dy = 10;
        elseif (x_vec_next(2) == s.data(t+1,2))
            R_py = 100;
            R_dy = 100;
        else
            R_py = -2;
            R_dy = -2;
        end
        
        if (x_vec_next(3)>(s.data(t+1,3)-state_lim)) && (x_vec_next(3)<(s.data(t+1,3)+state_lim))
            R_pz = 10;
            R_dz = 10;
        elseif (x_vec_next(3) == s.data(t+1,3))
            R_pz = 100;
            R_dz = 100;
        else
            R_pz = -2;
            R_dz = -2;
        end
        
        if (omega_vec_next(1)>(s.data(t+1,4)-state_lim)) && (omega_vec_next(1)<(s.data(t+1,4)+state_lim))
            R_pphi = 10;
            R_dphi = 10;
        elseif (omega_vec_next(1) == s.data(t+1,4))
            R_pphi = 100;
            R_dphi = 100;
        else
            R_pphi = -2;
            R_dphi = -2;
        end
        
        if (omega_vec_next(2)>(s.data(t+1,5)-state_lim)) && (omega_vec_next(2)<(s.data(t+1,5)+state_lim))
            R_ptheta = 10;
            R_dtheta = 10;
        elseif (omega_vec_next(2) == s.data(t+1,5))
            R_ptheta = 100;
            R_dtheta = 100;
        else
            R_ptheta = -2;
            R_dtheta = -2;
        end
        
        if (omega_vec_next(3)>(s.data(t+1,6)-state_lim)) && (omega_vec_next(3)<(s.data(t+1,6)+state_lim))
            R_ppsi = 10;
            R_dpsi = 10;
        elseif (omega_vec_next(3) == s.data(t+1,6))
            R_ppsi = 100;
            R_dpsi = 100;
        else
            R_ppsi = -2;
            R_dpsi = -2;
        end
        
        
        S_pos(1:3,t+1) = x_vec_next(1:3,1);
        S_pos(4:6,t+1) = x_vec_next(4:6,1);
        %
        S_alt(1:3,t+1) = omega_vec_next(1:3,1);
        S_alt(4:6,t+1) = omega_vec_next(4:6,1);
        %
        %

        % Discritization of S_t+1 states
        % Position X
        if x_vec_next(1) < X_min(1)
            n_x1 = 1;
        elseif x_vec_next(1) > X_max(1)
            n_x1 = 26;
        else
            n_x1 = floor(abs(((x_vec_next(1)/(X_max(1)-X_min(1)))*N)))+1;
        end
        % Position Y
        if x_vec_next(2) < X_min(2)
            n_y1 = 1;
        elseif x_vec_next(2) > X_max(2)
            n_y1 = 26;
        else
            n_y1 = floor(abs(((x_vec_next(2)/(X_max(2)-X_min(2)))*N)))+1;
        end
        % Position Z
        if x_vec_next(3) < X_min(3)
            n_z1 = 1;
        elseif x_vec_next(3) > X_max(3)
            n_z1 = 26;
        else
            n_z1 = floor(abs(((x_vec_next(3)/(X_max(3)-X_min(3)))*N)))+1;
        end
        % Velocity along X
        if x_vec_next(4) < X_dot_min(1)
            n_x_dot1 = 1;
        elseif x_vec_next(4) > X_dot_max(1)
            n_x_dot1 = 26;
        else
            n_x_dot1 = floor(abs(((x_vec_next(4)/(X_dot_max(1)-X_dot_min(1))))*N))+1;
        end
        % Velocity along Y
        if x_vec_next(5) < X_dot_min(2)
            n_y_dot1 = 1;
        elseif x_vec_next(5) > X_dot_max(2)
            n_y_dot1 = 26;
        else
            n_y_dot1 = floor(abs(((x_vec_next(5)/(X_dot_max(2)-X_dot_min(2)))*N)))+1;
        end
        % Velocity along Z
        if x_vec_next(6) < X_dot_min(3)
            n_z_dot1 = 1;
        elseif x_vec_next(6) > X_dot_max(3)
            n_z_dot1 = 26;
        else
            n_z_dot1 = floor(abs(((x_vec_next(6)/(X_dot_max(3)-X_dot_min(3)))*N)))+1;
        end
        
        % Orientation Phi
        if omega_vec_next(1) < omega_min(1)
            n_phi1 = 1;
        elseif omega_vec_next(1) > omega_max(1)
            n_phi1 = 26;
        else
            n_phi1 = floor(abs(((omega_vec_next(1)/(omega_max(1)-omega_min(1)))*N)))+1;
        end
        % Orientation Theta
        if omega_vec_next(2) < omega_min(2)
            n_theta1 = 1;
        elseif omega_vec_next(2) > omega_max(2)
            n_theta1 = 26;
        else
            n_theta1 = floor(abs(((omega_vec_next(2)/(omega_max(2)-omega_min(2)))*N)))+1;
        end
        % Orientation Psi
        if omega_vec_next(3) < omega_min(3)
            n_psi1 = 1;
        elseif omega_vec_next(3) > omega_max(3)
            n_psi1 = 26;
        else
            n_psi1 = floor(abs(((omega_vec_next(3)/(omega_max(3)-omega_min(3)))*N)))+1;
        end
        % Angular velocity about x-axis
        if omega_vec_next(4) < omega_dot_min(1)
            n_phi_dot1 = 1;
        elseif omega_vec_next(4) > omega_dot_max(1)
            n_phi_dot1 = 26;
        else
            n_phi_dot1 = floor(abs(((omega_vec_next(4)/(omega_dot_max(1)-omega_dot_min(1)))*N)))+1;
        end
        % Angular velocity about y-axis
        if omega_vec_next(5) < omega_dot_min(2)
            n_theta_dot1 = 1;
        elseif omega_vec_next(5) > omega_dot_max(2)
            n_theta_dot1 = 26;
        else
            n_theta_dot1 = floor(abs(((omega_vec_next(5)/(omega_dot_max(2)-omega_dot_min(2)))*N)))+1;
        end
        % Angular velocity about z-axis
        if omega_vec_next(6) < omega_dot_min(3)
            n_psi_dot1 = 1;
        elseif omega_vec_next(6) > omega_dot_max(3)
            n_psi_dot1 = 26;
        else
            n_psi_dot1 = floor(abs(((omega_vec_next(6)/(omega_dot_max(3)-omega_dot_min(3)))*N)))+1;
        end
    
        
    % Updating the Q Matrix for each state
    Q_1(n_x,ind1) = Q_1(n_x,ind1) + alpha_1*(R_px+(df*max(Q_1(n_x1,:)))-Q_1(n_x,ind1));
    Q_2(n_y,ind2) = Q_2(n_y,ind2) + alpha_1*(R_py+(df*max(Q_2(n_y1,:)))-Q_2(n_y,ind2));
    Q_3(n_z,ind3) = Q_3(n_z,ind3) + alpha_1*(R_pz+(df*max(Q_3(n_z1,:)))-Q_3(n_z,ind3));
    Q_4(n_phi,ind4) = Q_4(n_phi,ind4) + alpha_1*(R_pphi+(df*max(Q_4(n_phi1,:)))-Q_4(n_phi,ind4));
    Q_5(n_theta,ind5) = Q_5(n_theta,ind5) + alpha_1*(R_ptheta+(df*max(Q_5(n_theta1,:)))-Q_5(n_theta,ind5));
    Q_6(n_psi,ind6) = Q_6(n_psi,ind6) + alpha_1*(R_ppsi+(df*max(Q_6(n_psi1,:)))-Q_6(n_psi,ind6));
    
    Q_7(n_x_dot,ind7) = Q_7(n_x_dot,ind7) + alpha_2*(R_dx+(df*max(Q_7(n_x_dot1,:)))-Q_7(n_x_dot,ind7));
    Q_8(n_y_dot,ind8) = Q_8(n_y_dot,ind8) + alpha_2*(R_dy+(df*max(Q_8(n_y_dot1,:)))-Q_8(n_y_dot,ind8));
    Q_9(n_z_dot,ind9) = Q_9(n_z_dot,ind9) + alpha_2*(R_dz+(df*max(Q_9(n_z_dot1,:)))-Q_9(n_z_dot,ind9));
    Q_10(n_phi_dot,ind10) = Q_10(n_phi_dot,ind10) + alpha_2*(R_dphi+(df*max(Q_10(n_phi_dot1,:)))-Q_10(n_phi_dot,ind10));
    Q_11(n_theta_dot,ind11) = Q_11(n_theta_dot,ind11) + alpha_2*(R_dtheta+(df*max(Q_11(n_theta_dot1,:)))-Q_11(n_theta_dot,ind11));
    Q_12(n_psi_dot,ind12) = Q_12(n_psi_dot,ind12) + alpha_2*(R_dpsi+(df*max(Q_12(n_psi_dot1,:)))-Q_12(n_psi_dot,ind12));
    
    
    end
    episode = episode + 1;  % Incrementing the episode
end

% Saving the Q Matrices, kP and kD vectors as a file for PID Control
Q(:,:,1) = Q_1;
Q(:,:,2) = Q_2;
Q(:,:,3) = Q_3;
Q(:,:,4) = Q_4;
Q(:,:,5) = Q_5;
Q(:,:,6) = Q_6;
Q(:,:,7) = Q_7;
Q(:,:,8) = Q_8;
Q(:,:,9) = Q_9;
Q(:,:,10) = Q_10;
Q(:,:,11) = Q_11;
Q(:,:,12) = Q_12;

kP(:,1) = kPx_vec;
kP(:,2) = kPy_vec;
kP(:,3) = kPz_vec;
kP(:,4) = kPphi_vec;
kP(:,5) = kPtheta_vec;
kP(:,6) = kPpsi_vec;

kD(:,1) = kDx_vec;
kD(:,2) = kDy_vec;
kD(:,3) = kDz_vec;
kD(:,4) = kDphi_vec;
kD(:,5) = kDtheta_vec;
kD(:,6) = kDpsi_vec;

save('Q_matrices','Q');
save('kP_vectors','kP');
save('kD_vectors','kD');