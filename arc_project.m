% Time steps
time_steps = 0:0.1:2.5;

% Initialize states
S_t = zeros(12,1);

% Initialize Q tables for P and D
Q_1 = zeros(12, length(time_steps));
Q_2 = zeros(12, length(time_steps));
Q_3 = zeros(12, length(time_steps));
Q_4 = zeros(12, length(time_steps));

e_p = zeros(6,length(time_steps));
e_d = zeros(6,length(time_steps));

% Learning rates
alpha_1 = 0.01;
alpha_2 = 0.01;

% Exploration rate
epsilon = 0.27;

% Episodes
max_episodes = 6001;

%
X_min = zeros(3,1);
X_max = 5 * ones(3,1);
%
omega_min = -45 * pi/180 * ones(3,1);
omega_max = 45 * pi/180 * ones(3,1);
%
X_dot_min = zeros(3,1);
X_dot_max = ones(3,1);
%
omega_dot_min = -15 * pi/180 * ones(3,1);
omega_dot_max = 15 * pi/180 * ones(3,1);
%
X_con = zeros(3,1);
X_dot_con = zeros(3,1);
omega_con = zeros(3,1);
omega_dot_con = zeros(3,1);
%
% Number of buckets each variable is defined into
N = 10;
%
% Discretization
n_x = zeros(3,length(time_steps));
n_x_dot = zeros(3,length(time_steps));
n_omega = zeros(3,length(time_steps));
n_omega_dot = zeros(3,length(time_steps));
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
pos_thresh = 0.01;
alt_thresh = 0.01;

% 
simout = sim('quadcopter_control_training','StartTime',num2str(0),'StopTime',...
    num2str(0.1),'OutputOption','SpecifiedOutputTimes','OutputTimes',num2str(0.1));
S_pos = zeros(6,length(time_steps));
S_alt = zeros(6,length(time_steps));

e_p_lim = 0.01;
e_d_lim = 0.1;

df = 0.75;


episode = 1;
while episode < max_episodes

    % Decay. 
    if episode > 0.6*max_episodes
        epsilon = 1 / (1 + exp(episode)) + 0.001;
    else
        epsilon = 0;
    end
    for t = 0:0.1:2.5
        % Discretization of states
        S_pos(1:3,t) = S_t(1:3);
        S_pos(4:6,t) = S_t(7:9);
        %
        S_alt(1:3,t) = S_t(4:6);
        S_alt(4:6,t) = S_t(10:12);
        %
        %
        if X_con < X_min
            n_x(:,t) = ones(3,1);
        elseif X_con > X_min
            n_x(:,t) = 10*ones(3,1);
        elseif (X_min <= X_con) && (X_con <= X_max)
            n_x(:,t) = (X_con .* N ./ (X_max - X_min)) + 1*ones(3,1); 
        end
        %
        %
        if X_dot_con < X_dot_min
            n_x_dot(:,t) = 1*ones(3,1);
        elseif X_dot_con > X_dot_min
            n_x_dot(:,t) = 10*ones(3,1);
        elseif (X_dot_min <= X_dot_con) && (X_dot_con <= X_dot_max)
            n_x_dot(:,t) = (X_dot_con .* N ./ (X_dot_max - X_dot_min)) + 1*ones(3,1); 
        end
        %
        %
        if omega_con < omega_min
            n_omega(:,t) = 1*ones(3,1);
        elseif omega_con > omega_min
            n_omega(:,t) = 10*ones(3,1);
        elseif (omega_min <= omega_con) && (omega_con <= omega_max)
            n_omega(:,t) = (omega_con .* N ./ (omega_max - omega_min)) + 1*ones(3,1); 
        end
        %
        %
        if omega_dot_con < omega_dot_min
            n_omega_dot(:,t) = 1*ones(3,1);
        elseif omega_dot_con > omega_dot_min
            n_omega_dot(:,t) = 10*ones(3,1);
        elseif (omega_dot_min <= omega_dot_con) && (omega_dot_con <= omega_dot_max)
            n_omega_dot(:,t) = (omega_dot_con .* N ./ (omega_dot_max - omega_dot_min)) + 1*ones(3,1); 
        end
        %
        %
        for i = 1:1:2
            % xi
            x_i = rand;
            %
            if x_i < epsilon
                action_pos_p(:,t) = rand(Q_1);
            else
                action_pos_p(:,t) = max(Q_1);
            end
            %
            %
            if x_i < epsilon
                action_alt_p(:,t) = rand(Q_2);
            else
                action_alt_p(:,t) = max(Q_2);
            end
        end
        for i  = 3:4
            % xi
            x_i = rand;
            %
            if x_i < epsilon
                action_pos_d(:,t) = rand(Q_3);
            else
                action_pos_d(:,t) = max(Q_3);
            end
            %
            %
            if x_i < epsilon
                action_alt_d(:,t) = rand(Q_4);
            else
                action_alt_d(:,t) = max(Q_4);
            end
        end
        % Obtain total output u(t)
        % u_p = k_p(e) + k_d(e_dot)     
        % e = x_d - x_c   e_dot = x_dot_d -x_dot_c
        %
        % u_d = k_p(e) + k_d(e_dot)
        % e = phi_d - phi_c     e_dot = phi_dot_d - phi_dot_c
        %
        x_ref = zeros(6,1);
        x_ref(1:3) = x_vector(1:3);
        x_ref(4:6) = x_vector_dot(1:3);
        e_p(:,t) = x_ref - S_pos;
        
        omega_ref = zeros(6,1);
        omega_ref(1:3) = theta_vector(1:3);
        omega_ref(4:6) = theta_vector_dot(1:3);
        e_d(:,t) = omega_ref - S_alt;
        
        u_p(:,t) = (k_p*e_p(1:3,t)) + (k_d*e_p(4:6,t));
        u_d(:,t) = (k_p*e_d(1:3,t)) + (k_d*e_d(4:6,t));
        
        %
        % Observe new state S_t+1
        S_pos(:,t+1) = S_t(1:3,t);
        S_pos(:,t+1) = S_t(7:9,t);
        %
        S_alt(:,t+1) = S_t(4:6,t);
        S_alt(:,t+1) = S_t(10:12,t);
        %
        % Receive reward R_p for Q_1 and Q_2
        %
        
        x0 = 
        
        
        
        simout = sim('quadcopter_control_training','StartTime',num2str(0),'StopTime',...
            num2str(0.1),'OutputOption','SpecifiedOutputTimes','OutputTimes',num2str(0.1));
        
        x_vec_next(1:3,1) = (simout.simout.Data(end,1:3))';
        x_vec_next(4:6,1) = (simout.simout.Data(end,7:9))';
        omega_vec_next(1:3,1) = (simout.simout.Data(end,4:6))';
        omega_vec_next(4:6,1) = (simout.simout.Data(end,10:12))';
        X_con(1:3,1) = x_vec_next(1:3,1);
        X_dot_con(1:3,1) = x_vec_next(4:6,1);
        omega_con(1:3,1) = omega_vec_next(1:3,1);
        omega_dot_con(1:3,1) = omega_vec_next(4:6,1);
        e_p_next = x_vec_next - S_pos(:,t+1);
        e_d_next = omega_vec_next - S_alt(:,t+1);
        
        if (abs(e_p_next)<abs(e_p(:,t))) && (abs(e_p_next)>e_p_lim)
            R_p = 0;
        elseif (abs(e_p_next)<=e_p_lim)
            R_p = 1;
        else
            R_p = -1;
        end
        
        
        if (abs(e_d_next)<abs(e_d(:,t))) && (abs(e_d_next)>e_d_lim)
            R_d = 0;
        elseif (abs(e_d_next)<=e_d_lim)
            R_d = 1;
        else
            R_d = -1;
        end
        
        S_pos(1:3,t+1) = S_t(1:3);
        S_pos(4:6,t+1) = S_t(7:9);
        %
        S_alt(1:3,t+1) = S_t(4:6);
        S_alt(4:6,t+1) = S_t(10:12);
        %
        %
        if X_con < X_min
            n_x(:,t+1) = ones(3,1);
        elseif X_con > X_min
            n_x(:,t+1) = 10*ones(3,1);
        elseif (X_min <= X_con) && (X_con <= X_max)
            n_x(:,t+1) = (X_con .* N ./ (X_max - X_min)) + 1*ones(3,1); 
        end
        
        if X_dot_con < X_dot_min
            n_x_dot(:,t+1) = 1*ones(3,1);
        elseif X_dot_con > X_dot_min
            n_x_dot(:,t+1) = 10*ones(3,1);
        elseif (X_dot_min <= X_dot_con) && (X_dot_con <= X_dot_max)
            n_x_dot(:,t+1) = (X_dot_con .* N ./ (X_dot_max - X_dot_min)) + 1*ones(3,1); 
        end
        %
        %
        if omega_con < omega_min
            n_omega(:,t+1) = 1*ones(3,1);
        elseif omega_con > omega_min
            n_omega(:,t+1) = 10*ones(3,1);
        elseif (omega_min <= omega_con) && (omega_con <= omega_max)
            n_omega(:,t+1) = (omega_con .* N ./ (omega_max - omega_min)) + 1*ones(3,1); 
        end
        %
        %
        if omega_dot_con < omega_dot_min
            n_omega_dot(:,t+1) = 1*ones(3,1);
        elseif omega_dot_con > omega_dot_min
            n_omega_dot(:,t+1) = 10*ones(3,1);
        elseif (omega_dot_min <= omega_dot_con) && (omega_dot_con <= omega_dot_max)
            n_omega_dot(:,t+1) = (omega_dot_con .* N ./ (omega_dot_max - omega_dot_min)) + 1*ones(3,1); 
        end
    
    end
    
    Q_1(:,t) = (Q_1(:,t) + (alpha_1 * R_p)) + (df * max(Q_1(:,t+1)-Q_1(:,t)));
    Q_2(:,t) = (Q_2(:,t) + (alpha_1 * R_p)) + (df * max(Q_2(:,t+1)-Q_2(:,t)));
    Q_3(:,t) = (Q_3(:,t) + (alpha_2 * R_d)) + (df * max(Q_3(:,t+1)-Q_3(:,t)));
    Q_4(:,t) = (Q_4(:,t) + (alpha_2 * R_d)) + (df * max(Q_4(:,t+1)-Q_4(:,t)));
    
    episode = episode + 1;
end