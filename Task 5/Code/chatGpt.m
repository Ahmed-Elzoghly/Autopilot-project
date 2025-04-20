%% === Build outer structure of Longitudinal_Test ===
model = 'Longitudinal_Test';
new_system(model); 
open_system(model);

%% 1) Inputs
add_block('simulink/Sources/Constant', [model '/thetacom_deg'], ...
    'Position',[30  50  60  70],'Value','0');
add_block('simulink/Sources/Constant', [model '/ucom'], ...
    'Position',[30 120  60 140],'Value','0');

%% 2) Command bus
add_block('simulink/Signal Routing/Bus Creator', [model '/Command'], ...
    'Inputs','2', ...
    'Position',[120  50 150 140]);
add_line(model, 'thetacom_deg/1','Command/1','autorouting','on');
add_line(model, 'ucom/1',        'Command/2','autorouting','on');

%% 3) Three main subsystems
add_block('simulink/Ports & Subsystems/Subsystem', [model '/Pitch & Velocity Controller'], ...
    'Position',[200  50 350 200]);
add_block('simulink/Ports & Subsystems/Subsystem', [model '/Longitudinal'], ...
    'Position',[400  50 550 200]);
add_block('simulink/Ports & Subsystems/Subsystem', [model '/Displays2'], ...
    'Position',[600  50 750 200]);

%% 4) control_actions and delta_states buses
add_block('simulink/Signal Routing/Bus Creator', [model '/control_actions'], ...
    'Inputs','2','Position',[370  80 390 140]);
add_block('simulink/Signal Routing/Bus Creator', [model '/delta_states'], ...
    'Inputs','4','Position',[570  80 590 160]);

%% 5) Wire it all up

% Command → Pitch & Vel Ctrl
add_line(model, 'Command/1','Pitch & Velocity Controller/1','autorouting','on');

% Pitch & Vel Ctrl → control_actions bus
add_line(model, 'Pitch & Velocity Controller/1','control_actions/1','autorouting','on');

% control_actions bus → Longitudinal
add_line(model, 'control_actions/1','Longitudinal/1','autorouting','on');

% Longitudinal → delta_states bus
add_line(model, 'Longitudinal/1','delta_states/1','autorouting','on');

% delta_states bus feeds back into Pitch & Vel Ctrl and also into Displays2
add_line(model, 'delta_states/1','Pitch & Velocity Controller/2','autorouting','on');
add_line(model, 'delta_states/1','Displays2/3','autorouting','on');

% And hook command & control_actions into Displays2
add_line(model, 'Command/1',              'Displays2/1','autorouting','on');
add_line(model, 'control_actions/1',      'Displays2/2','autorouting','on');

%% 6) Finish up
Simulink.BlockDiagram.arrangeSystem(model);
save_system(model);
open_system(model);
