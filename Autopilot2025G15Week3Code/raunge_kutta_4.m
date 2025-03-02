function states_next = raunge_kutta_4(t, Forces, Moments, states, mass, Inertia, dt)
    K1 = get_states_dot(t, states, Forces, Moments, mass, Inertia);
    K2 = get_states_dot(t + 0.5*dt, states + 0.5*dt*K1, Forces, Moments, mass, Inertia);
    K3 = get_states_dot(t + 0.5*dt, states + 0.5*dt*K2, Forces, Moments, mass, Inertia);
    K4 = get_states_dot(t + dt, states + dt*K3, Forces, Moments, mass, Inertia);

    states_next = states + (dt/6) * (K1 + 2*K2 + 2*K3 + K4);
end
