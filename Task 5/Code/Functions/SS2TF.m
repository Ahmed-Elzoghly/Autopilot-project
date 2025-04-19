function [sys,TF_ij] = SS2TF(A,B,C,D,j,i)
%% Discribtion:
%{  
                    Gereralization:

For:
number of input = n;
number of output = m;

A=rand(m,m);
B=rand(m,n);
C=rand(n,m);
D=rand(n,n);
sys =ss(A,B,C,D)

To obtain transfer function from certain input(j) to certain output(i):
Start_Row = (j-1)*n+1;
End_Row = j*n;
Start_Col = (i-1)*m+1;
End_Col = i*m;

sys_ij = @(i,j) sys(Start_Row:End_Row, Start_Col:End_Col); 
TF_ij = tf(sys_ij);

*E.g. Extract TF from the 4nd input to 2th output (state)
sys_24 = tf(sys(4,2))


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
This function takes 
        ** The State Space Matrices A,B,C,D
        ** The desired input(i) and output(j)

and gives 
        ** Desired State Space Model
        ** Desired Transfer Function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%}
%}
%% Main Fn:

[m,n] = size(B);


Start_Row = (j-1)*n+1;
End_Row = j*n;
Start_Col = (i-1)*m+1;
End_Col = i*m;

system =ss(A,B,C,D);

sys = @(i,j) system(Start_Row:End_Row, Start_Col:End_Col); 

% e.g. Extract TF from the 2nd input to 4th output
TF_ij = tf(system(i,j));


end

