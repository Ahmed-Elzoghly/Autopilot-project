function [F, M] = Airframe_Model(States0,Current_States,wdot,Mass,I,g,input, Matrix_states, Matrix_controls)

   
    wdot0 = 0;
    I_vec = transpose([I(1), I(5), I(9)]);
   
    d_u    = Current_States(1)-States0(1); 
    d_v    = Current_States(2)-States0(2);
    d_w    = Current_States(3)-States0(3);
    d_p    = Current_States(4)-States0(4);
    d_q    = Current_States(5)-States0(5);
    d_r    = Current_States(6)-States0(6);
    d_wdot = wdot-wdot0;
    d_aileron   = input(1); 
    d_rudder    = input(2);
    d_elevator  = input(3);
    d_thrust    = input(4);
    
    delta_states = [d_u d_v d_w d_p d_q d_r d_wdot]';
    
                 
    delta_controls = [d_aileron d_elevator d_thrust d_rudder]';             
                 
    
    delta=Matrix_states*delta_states + Matrix_controls*delta_controls;
    
    delta_F =  delta(1:3)*Mass;
    delta_M =  I_vec .* delta(4:6);
    
    gravity0 = [Mass*g*sin(States0(8))
                -Mass*g*cos(States0(8))*sin(States0(7))
                -Mass*g*cos(States0(8))*cos(States0(7))];
       
    gravity = [-Mass*g*sin(Current_States(8))
               Mass*g*cos(Current_States(8))*sin(Current_States(7))
               Mass*g*cos(Current_States(8))*cos(Current_States(7))];
           
           
    F = delta_F + gravity0 + gravity;
    
    M =delta_M ;
end