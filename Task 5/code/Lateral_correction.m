function SD_Lateral_Final = Lateral_correction(SD_Lat_dash, Vto, Ix, Iz, Ixz)
    
    G = 1 / (1 - (Ixz^2 / (Ix * Iz)));
%%% Drivatives dashed 
  Yv = SD_Lat_dash(1);
  Yb = SD_Lat_dash(2);
  L_beta_dash = SD_Lat_dash(3);
  N_beta_dash = SD_Lat_dash(4);
  L_p_dash    = SD_Lat_dash(5);
  N_p_dash    = SD_Lat_dash(6);
  L_r_dash    = SD_Lat_dash(7);
  N_r_dash    = SD_Lat_dash(8);
  Y_star_da   = SD_Lat_dash(9);
  Y_star_dr   = SD_Lat_dash(10);
  L_da_dash   = SD_Lat_dash(11);
  N_da_dash   = SD_Lat_dash(12);
   L_dr_dash  = SD_Lat_dash(13);
  N_dr_dash   = SD_Lat_dash(14);
  %%% undashed drivatives
  M =([G Ixz*G/Ix ;Ixz*G/Iz G]); %% the matrix used to transfer from dashed to undashed
  M_inv = inv(M) ;
  %%%
  A = M_inv * [ L_beta_dash ; N_beta_dash ] ;
  L_beta = A(1);
  N_beta = A(2);
  %%%
 B = M_inv * [L_p_dash ;  N_p_dash  ] ;
  L_p = B(1);
  N_p = B(2); 
  %%%
  C = M_inv * [L_r_dash ;  N_r_dash  ] ;
  L_r = C(1);
  N_r = C(2);
  %%%
  D = M_inv*[L_da_dash ; N_da_dash ] ;  %%% A B C D E just symboles
  L_da = D(1);
  N_da = D(2) ;
  %%%
  E = M_inv*[L_dr_dash ; N_dr_dash ] ;
  L_dr = E(1);
  N_dr = E(2);
  %%%
  Y_da = Y_star_da*Vto ;
  %%%
  Y_dr   = Y_star_dr *Vto ;
  
%%%%
    SD_Lateral_Final = [Yv, Yb ,L_beta, N_beta ,L_p ,N_p , L_r ,N_r,Y_da , Y_dr, L_da , N_da , L_dr , N_dr , ];
end