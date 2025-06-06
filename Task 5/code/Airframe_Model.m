function [F, M] = Airframe_Model(States0,Current_States,wdot,Mass,I,g,input, Matrix_states, Matrix_controls)

    
    % Ixx = I(1);
    % Iyy = I(5);
    % Izz = I(9);
    wdot0 = 0;
    I_vec = transpose([I(1), I(5), I(9)]);
    
    % Xu    = long_der(1);
    % Xw    = long_der(4);
    % Xde   = long_der(11);
    % Xdth   = long_der(14);
    % Zu    = long_der(2);
    % Zw    = long_der(5);
    % Zwd = long_der(7);
    % Zq    = long_der(8);
    % Zde   = long_der(12);
    % Zdth   = long_der(15);
    % Mu    = long_der(3);
    % Mw    = long_der(6);
    % Mwd = long_der(9);
    % Mq    = long_der(10);
    % Mde   = long_der(13);
    % Mdth   = long_der(16);
    % 
    % 
    % Yv  = Lat_der(1); 
    % Yp  = 0.2773;
    % Yr  = 0.7618;
    % Lv  = Lat_der(2);
    % Nv = Lat_der(3);
    % Lp  = Lat_der(4);
    % Np  = Lat_der(5);
    % Lr = Lat_der(6);
    % Nr = Lat_der(7);
    % Yda  = Lat_der(8);
    % Ydr  = Lat_der(9);
    % Lda  = Lat_der(10);
    % Nda = Lat_der(11);
    % Ldr = Lat_der(12);
    % Ndr  = Lat_der(13);
   
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
    
    % Matrix_states = [Xu   0   Xw   0   0   0   0 
    %                  0    Yv  0    Yp  0   Yr  0
    %                  Zu   0   Zw   0   Zq  0   Zwd
    %                  0    Lv  0    Lp  0   Lr  0
    %                  Mu   0   Mw   0   Mq  0   Mwd
    %                  0    Nv  0    Np  0   Nr  0];
                 
    delta_controls = [d_aileron d_elevator d_thrust d_rudder]';             
                 
    % Matrix_controls = [0    Xde   Xdth   0
    %                    Yda  0     0      Ydr
    %                    0    Zde   Zdth   0
    %                    Lda  0     0      Ldr
    %                    0    Mde   Mdth   0
    %                    Nda  0     0      Ndr];
    
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