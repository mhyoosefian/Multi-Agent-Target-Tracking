function new_state = leader_dynamics(old_state, w, Ts)

F = [eye(2) Ts*eye(2)
     zeros(2) eye(2)];

% Computing the new state
new_state = F*old_state + w;