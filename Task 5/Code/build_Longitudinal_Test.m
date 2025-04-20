%% === Script to generate Longitudinal_Test ===
model = 'Longitudinal_Test';

% 1) Create & open model
new_system(model);
open_system(model);

%% 2) Top‑Level: Inports & Buses
% Inports
add_block('simulink/Sources/In1',    [model '/thetacom_deg'], 'Position',[30  40  60  60],'Port','1');
add_block('simulink/Sources/In1',    [model '/ucom'],         'Position',[30 100  60 120],'Port','2');
add_block('simulink/Sources/In1',    [model '/delta_states'], 'Position',[30 160  60 180],'Port','3');

% Command bus
add_block('simulink/Signal Routing/Bus Creator', [model '/Command'], ...
    'Inputs','2','Position',[120  60  150 110]);
add_line(model, 'thetacom_deg/1','Command/1','autorouting','on');
add_line(model, 'ucom/1',        'Command/2','autorouting','on');

% control_actions bus (feed back into Longitudinal & out to Displays2)
add_block('simulink/Signal Routing/Bus Creator', [model '/control_actions'], ...
    'Inputs','2','Position',[450  50  480 100]);

% delta_states bus (output of Longitudinal into both Display & PitchCtrl)
add_block('simulink/Signal Routing/Bus Creator', [model '/delta_states_out'], ...
    'Inputs','4','Position',[800 150 830 270]);

%% 3) Pitch & Velocity Controller Subsystem
pvc = [model '/Pitch & Velocity Controller'];
add_block('simulink/Ports & Subsystems/Subsystem', pvc, 'Position',[200  20  400 200]);
% connect Command and delta_states into it
add_line(model, 'Command/1',       'Pitch & Velocity Controller/1','autorouting','on');
add_line(model, 'delta_states/1',   'Pitch & Velocity Controller/2','autorouting','on');
% connect its output to control_actions bus
add_line(model, 'Pitch & Velocity Controller/1', 'control_actions/1','autorouting','on');

% --- populate inside PVC
open_system(pvc);
% Inports
add_block('simulink/Ports & Subsystems/In1', [pvc '/Command'],       'Position',[30  40  60  60],'Port','1');
add_block('simulink/Ports & Subsystems/In1', [pvc '/delta_states'], 'Position',[30 100  60 120],'Port','2');

% Bus‑Selector: extract thetacom & delta_theta
add_block('simulink/Signal Routing/Bus Selector', [pvc '/BusSel_Cmd'], ...
    'OutputSignals','thetacom', 'Position',[100 40 150 70]);
add_block('simulink/Signal Routing/Bus Selector', [pvc '/BusSel_ds'], ...
    'OutputSignals','delta_theta', 'Position',[100 100 150 130]);

add_line(pvc,'Command/1','BusSel_Cmd/1','autorouting','on');
add_line(pvc,'delta_states/1','BusSel_ds/1','autorouting','on');

% D2R
add_block('simulink/Commonly Used Blocks/Gain', [pvc '/D2R'], ...
    'Gain','pi/180','Position',[180  45 220  75]);
add_line(pvc,'BusSel_Cmd/1','D2R/1','autorouting','on');

% Sum: thetacom(rad) - delta_theta
add_block('simulink/Math Operations/Sum',[pvc '/Sum_err'], ...
    'Inputs','+-','Position',[260  45 300  75]);
add_line(pvc,'D2R/1','Sum_err/1','autorouting','on');
add_line(pvc,'BusSel_ds/1','Sum_err/2','autorouting','on');

% PI controller (as TF: (Kp s + Ki)/s)
add_block('simulink/Continuous/Transfer Fcn',[pvc '/PI_pitch'], ...
    'Numerator','[Kp Ki]','Denominator','[1 0]','Position',[360 45 430 75]);

add_line(pvc,'Sum_err/1','PI_pitch/1','autorouting','on');

% Feedback summing junction: + (PI)  – (q feedback)
add_block('simulink/Math Operations/Sum',[pvc '/Sum_fb'], ...
    'Inputs','+-','Position',[480 45 520 75]);
add_block('simulink/Commonly Used Blocks/Gain',[pvc '/Gain_q'], ...
    'Gain','0.71351','Position',[360 100 400 130]);
% extract delta_q from delta_states
add_block('simulink/Signal Routing/Bus Selector', [pvc '/BusSel_q'], ...
    'OutputSignals','delta_q','Position',[260 100 300 130]);

add_line(pvc,'delta_states/1','BusSel_q/1','autorouting','on');
add_line(pvc,'BusSel_q/1','Gain_q/1','autorouting','on');
add_line(pvc,'Gain_q/1','Sum_fb/2','autorouting','on');
add_line(pvc,'PI_pitch/1','Sum_fb/1','autorouting','on');

% Negate (–1)
add_block('simulink/Commonly Used Blocks/Gain',[pvc '/Neg1'], ...
    'Gain','-1','Position',[550 45 590 75]);
add_line(pvc,'Sum_fb/1','Neg1/1','autorouting','on');

% Servo dynamics
add_block('simulink/Continuous/Transfer Fcn',[pvc '/servo'], ...
    'Numerator','1.2983','Denominator','[1 0.2968]','Position',[650 45 720 75]);
add_line(pvc,'Neg1/1','servo/1','autorouting','on');

% Saturation
add_block('simulink/Commonly Used Blocks/Saturation',[pvc '/Saturation'], ...
    'UpperLimit','inf','LowerLimit','-inf','Position',[780 45 820 75]);
add_line(pvc,'servo/1','Saturation/1','autorouting','on');

% Outport
add_block('simulink/Ports & Subsystems/Out1',[pvc '/delta_elevator'], ...
    'Position',[900 55 930 75],'Port','1');
add_line(pvc,'Saturation/1','delta_elevator/1','autorouting','on');

close_system(pvc);

%% 4) Longitudinal Subsystem
longs = [model '/Longitudinal'];
add_block('simulink/Ports & Subsystems/Subsystem', longs, 'Position',[200 250 400 400]);
% connect control_actions & delta_thrust into it
add_line(model,'control_actions/1','Longitudinal/1','autorouting','on');
add_line(model,'control_actions/2','Longitudinal/2','autorouting','on');
% connect its output to delta_states_out bus
add_line(model,'Longitudinal/1','delta_states_out/1','autorouting','on');

open_system(longs);
% Inports
add_block('simulink/Ports & Subsystems/In1',[longs '/delta_elev'],'Position',[30 60 60 80],'Port','1');
add_block('simulink/Ports & Subsystems/In1',[longs '/delta_thrust'],'Position',[30 120 60 140],'Port','2');
% Mux into vector
add_block('simulink/Signal Routing/Mux',[longs '/Mux_u_e'], ...
    'Inputs','2','Position',[100 80 140 140]);
add_line(longs,'delta_elev/1','Mux_u_e/1','autorouting','on');
add_line(longs,'delta_thrust/1','Mux_u_e/2','autorouting','on');
% State‑space block
add_block('simulink/Continuous/State-Space',[longs '/Long_SS'], ...
    'A','A','B','B','C','C','D','D','Position',[200 80 300 160]);
add_line(longs,'Mux_u_e/1','Long_SS/2','autorouting','on');
% Demux outputs into 4 signals
add_block('simulink/Signal Routing/Demux',[longs '/Demux_ds'], ...
    'Outputs','4','Position',[350 80 430 160]);
add_line(longs,'Long_SS/1','Demux_ds/1','autorouting','on');
% Outports for delta_states components
for i=1:4
    add_block('simulink/Ports & Subsystems/Out1', ...
        sprintf('%s/ds%d',longs,i), ...
        'Position',[480  40+30*i 510  60+30*i],'Port',num2str(i));
    add_line(longs, sprintf('Demux_ds/%d',i), sprintf('ds%d/1',i),'autorouting','on');
end
close_system(longs);

%% 5) Displays2 Subsystem
disp2 = [model '/Displays2'];
add_block('simulink/Ports & Subsystems/Subsystem', disp2, 'Position',[600  20 820 200]);
% connect Command, control_actions, delta_states
add_line(model,'Command/1',       'Displays2/1','autorouting','on');
add_line(model,'control_actions/1','Displays2/2','autorouting','on');
add_line(model,'delta_states_out/1','Displays2/3','autorouting','on');

open_system(disp2);
% Inports
add_block('simulink/Ports & Subsystems/In1',[disp2 '/Command'],        'Position',[30  40 60  60],'Port','1');
add_block('simulink/Ports & Subsystems/In1',[disp2 '/delta_elev'],     'Position',[30 100 60 120],'Port','2');
add_block('simulink/Ports & Subsystems/In1',[disp2 '/delta_thrust'],   'Position',[30 160 60 180],'Port','3');
add_block('simulink/Ports & Subsystems/In1',[disp2 '/delta_states'],   'Position',[30 220 60 240],'Port','4');

% (Within here you can add all your R2D, Scopes, displays exactly as in your screenshot.)
% For brevity: show one R2D + Scope example:

add_block('simulink/Signal Routing/Bus Selector', [disp2 '/BusSel_cmd'], ...
    'OutputSignals','thetacom', 'Position',[100 40 140 70]);
add_line(disp2,'Command/1','BusSel_cmd/1','autorouting','on');

add_block('simulink/Commonly Used Blocks/Gain',[disp2 '/R2D'], ...
    'Gain','180/pi','Position',[180 40 220 70]);
add_line(disp2,'BusSel_cmd/1','R2D/1','autorouting','on');

add_block('simulink/Sinks/Scope',[disp2 '/Scope_theta'], ...
    'Position',[260 40 310 80]);
add_line(disp2,'R2D/1','Scope_theta/1','autorouting','on');

% ... you can continue adding all the rest of your converters, scopes and display blocks similarly.

close_system(disp2);

%% 6) Finalize
Simulink.BlockDiagram.arrangeSystem(model);
fprintf('Model "%s" has been created and opened.\n',model);
