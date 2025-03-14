function [t_vec, states_vec] = raunge_kutta_4(t_vec, y0, Forces, Moments, m, I_mat)

    n = length(t_vec);
    dt = t_vec(2) - t_vec(1);
    states_vec = zeros(12,n);
    states_vec(:,1) =y0;
    
    for i =1:n-1
        
        K1 = get_states_dot(t_vec(i),states_vec(:,i), Forces, Moments, m, I_mat);
        K2 = get_states_dot(t_vec(i)+.5*dt,states_vec(:,i)+K1*0.5*dt, Forces, Moments, m, I_mat);
        K3 = get_states_dot(t_vec(i)+.5*dt,states_vec(:,i)+K2*0.5*dt, Forces, Moments, m, I_mat);
        K4 = get_states_dot(t_vec(i)+dt,states_vec(:,i)+K3*dt, Forces, Moments, m, I_mat);
    
        states_vec(:,i+1) = states_vec(:,i) + dt/6 * (K1+2.*K2+2.*K3+K4);
    end
end