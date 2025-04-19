function states_dot = get_states_dot(t_vec, states_vec, Forces, Moments, m, I_mat, I_inv)
% get_states_dot Computes the time derivatives of the state variables
% 
% Inputs:
%   - states_vec: State vector [u, v, w, p, q, r, phi, theta, psi, x, y, z] (12x1)
%   - Forces: External forces in body frame [Fx, Fy, Fz] (3x1)
%   - Moments: External moments in body frame [Mx, My, Mz] (3x1)
%   - m: Mass of the vehicle (scalar)
%   - I_mat: Inertia matrix (3x3)
% 
% Outputs:
%   - states_dot: Time derivative of states (12x1)

 % Extract states
    u =     states_vec(1);
    v =     states_vec(2);
    w =     states_vec(3);
    p =     states_vec(4);
    q =     states_vec(5);
    r =     states_vec(6);
    phi =   states_vec(7);
    theta = states_vec(8);
    psi =   states_vec(9);
    x =     states_vec(10);
    y =     states_vec(11);
    z =     states_vec(12);

    % Rotation matrix for Euler angles (ZYX convention)
    J = [1, sin(phi) * tan(theta), cos(phi) * tan(theta);
         0, cos(phi), -sin(phi);
         0, sin(phi) / cos(theta), cos(phi) / cos(theta)];

    % Compute translational dynamics
    vel_dot = (1 / m) * Forces - cross([p; q; r], [u; v; w]);

    % Compute rotational dynamics
    omega_dot = I_inv * (Moments - cross([p; q; r], I_mat * [p; q; r]));

    % Compute Euler angle rates
    euler_dot = J * [p; q; r];

    % Compute position rates in the inertial frame
    pos_dot = eul2rotm([psi, theta, phi], 'ZYX') * [u; v; w];

    % Concatenate state derivatives
    states_dot = [vel_dot; omega_dot; euler_dot; pos_dot];
end
