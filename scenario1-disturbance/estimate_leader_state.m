function [l_state_hat_new, P_new] = estimate_leader_state(y, Ts, l_state_hat_old, P_old, f_pos, Q, R, A)

%{
INPUTS:     1- y: a cell including measurements from all agents. It could be a
            cell of empty elements. We assume we receive all measurements at the same
            time, i.e., there is no delay in measurements of various agents.
            2- Ts: simulation timestep.
            3- l_state_hat_old: a cell including the previous estimates of
            the leader state.
            4- P_old: a cell including all covariance matrices of previous
            estimations of the leader state.
            5- f_pos: a cell including the position of all followers.
            6- Q: covariance of process noise.
            7- R: a cell including all covariance matrices of all
            measuremets noise.
            8- A: the adjacency matrix of the communication graph. If the
            entry ij of the matrix is 1 it means that information flows
            from agent j to i. So, entries with value 1 in row i correspond
            to the agents that send info to agent i.

OUTPUTS:    1- l_state_hat_new: a cell including updated state of the
            leader for all agents.
            2- P_new: a cell including all updated covariance matrices for
            all followers.
%}
 


% Compute the Jacobians and noise covariances
F = [eye(2) Ts*eye(2)
     zeros(2) eye(2)];
num_agents = size(A, 1);
for i=1:num_agents
    Omega{i} = inv(P_old{1});               % the information matrix
    z{i} = Omega{i}*l_state_hat_old{i};     % the information vector
    V{i} = inv(R{i});
end
W = inv(Q);

% Old leader's position estimate
for i=1:num_agents
    q_hat{i} = l_state_hat_old{i}(1:2);
end

% The estimate of the measurements
for i=1:num_agents
    h_hat{i} = norm(q_hat{i} - f_pos{i}, 2);
end

% The weights for each agent
for i=1:num_agents
    c{i} = A(i, :)/sum(A(:, i));
end


% The update step. Do it when we have a measurement.
if ~isempty(y{1})
    for i=1:num_agents
        H{i} = [(q_hat{i}-f_pos{i})'/h_hat{i} 0 0];
        y_err{i} = y{i} - h_hat{i} + H{i}*l_state_hat_old{i};
        delta_z{i} = H{i}'*V{i}*y_err{i};
        delta_Omega{i}=H{i}'*V{i}*H{i};
    end
    
    for i=1:num_agents
        z_fus{i} = zeros(size(z{i}));
        Omega_fus{i} = zeros(size(Omega{i}));
        delta_z_fus{i} = zeros(size(delta_z{i}));
         delta_Omega_fus{i} = zeros(size(delta_Omega{i}));
        for j=1:num_agents
            z_fus{i} = z_fus{i} + c{i}(j)*z{j};
            Omega_fus{i} = Omega_fus{i} + c{i}(j)*Omega{j};
            delta_z_fus{i} = delta_z_fus{i} + c{i}(j)*delta_z{j};
             delta_Omega_fus{i} = delta_Omega_fus{i} + c{i}(j)*delta_Omega{j};
        end
    end
    
    
    for i=1:num_agents
        z_hat_update{i} = z_fus{i} + delta_z_fus{i};
        Omega_update{i} = Omega_fus{i} + delta_Omega_fus{i};
        l_state_hat_new{i} = inv(Omega_update{i})*z_hat_update{i};
    end

else
    for i=1:num_agents
        l_state_hat_new{i} = l_state_hat_old{i};
        Omega_update{i} = Omega{i};
    end
end

% The prediction step
for i=1:num_agents
    l_state_hat_new{i} = F*l_state_hat_new{i};
    Omega_predict{i} = W - W*F/(Omega_update{i} + F'*W*F)*F'*W;
    P_new{i} = inv(Omega_predict{i});
end
