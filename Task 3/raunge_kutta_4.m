function [states_next, wdot] = raunge_kutta_4(t, Forces, Moments, states, mass, Inertia, I_inv, dt)
    K1 = get_states_dot(t, states, Forces, Moments, mass, Inertia, I_inv);
    K2 = get_states_dot(t + 0.5*dt, states + 0.5*dt*K1, Forces, Moments, mass, Inertia, I_inv);
    K3 = get_states_dot(t + 0.5*dt, states + 0.5*dt*K2, Forces, Moments, mass, Inertia, I_inv);
    K4 = get_states_dot(t + dt, states + dt*K3, Forces, Moments, mass, Inertia, I_inv);

    states_next = states + (dt/6) * (K1 + 2*K2 + 2*K3 + K4);
    wdot = K1(3);
end
