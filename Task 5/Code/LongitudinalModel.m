%% --- Populate the Longitudinal subsystem interior ---
longs = 'Longitudinal_Test/Subsystem';
open_system(longs);

% 1) Clear out anything you added previously
delete_block(longs, 'delta_elev');
delete_block(longs, 'delta_thrust');
delete_block(longs, 'BusIn');
delete_block(longs, 'Constant0');
delete_block(longs, 'ManualSwitch1');
delete_block(longs, 'BusCreator_in');
delete_block(longs, 'Long_SS');
delete_block(longs, 'DemuxY');
delete_block(longs, 'BusCreator_out');

% 2) Inports
add_block('simulink/Ports & Subsystems/In1',  [longs '/delta_elev'],   ...
    'Position',[30  50  60  70],'Port','1');
add_block('simulink/Ports & Subsystems/In1',  [longs '/delta_thrust'], ...
    'Position',[30 120  60 140],'Port','2');

% 3) Constant zero
add_block('simulink/Sources/Constant', [longs '/Constant0'], ...
    'Value','0', 'Position',[100 120 140 140]);

% 4) Manual Switch (with control port)
add_block('simulink/Signal Routing/Manual Switch', [longs '/ManualSwitch1'], ...
    'Position',[180 120 220 140], ...
    'ShowOutputPort','on','ControlPort','on');

% 5) Wire up thrust → switch, zero → switch
add_line(longs, 'delta_thrust/1', 'ManualSwitch1/2','autorouting','on');
add_line(longs, 'Constant0/1',    'ManualSwitch1/1','autorouting','on');

% 6) Input Bus Creator (2‑element bus)
add_block('simulink/Signal Routing/Bus Creator', [longs '/BusCreator_in'], ...
    'Inputs','2', ...
    'OutputSignals','delta_elev,delta_thrust', ...
    'Position',[260  80 300 160]);

add_line(longs, 'delta_elev/1',      'BusCreator_in/1','autorouting','on');
add_line(longs, 'ManualSwitch1/1',   'BusCreator_in/2','autorouting','on');

% 7) State‑Space block
add_block('simulink/Continuous/State-Space', [longs '/Long_SS'], ...
    'A','A','B','B','C','C','D','D', ...
    'Position',[350  80 450 180]);

add_line(longs, 'BusCreator_in/1', 'Long_SS/2','autorouting','on');

% 8) Demux the output vector into four signals
add_block('simulink/Signal Routing/Demux', [longs '/DemuxY'], ...
    'Outputs','4', ...
    'Position',[500  80 580 200]);

add_line(longs, 'Long_SS/1', 'DemuxY/1','autorouting','on');

% 9) Output Bus Creator (4‑element bus)
add_block('simulink/Signal Routing/Bus Creator', [longs '/BusCreator_out'], ...
    'Inputs','4', ...
    'OutputSignals','delta_u,delta_w,delta_q,delta_theta', ...
    'Position',[650  60 690 260]);

% connect each demuxed line into the output bus
for idx = 1:4
    add_line(longs, sprintf('DemuxY/%d',idx), sprintf('BusCreator_out/%d',idx), ...
             'autorouting','on');
end

% 10) Close the subsystem
close_system(longs);

% 11) Re‑arrange for a neat layout
Simulink.BlockDiagram.arrangeSystem(longs);
