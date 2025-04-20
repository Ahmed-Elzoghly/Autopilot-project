% Create a new Simulink model
modelName = 'PitchControlModel';
new_system(modelName);
open_system(modelName);

% Set positions and add blocks
add_block('simulink/Sources/Constant', [modelName '/thetacom_deg'], 'Position', [30, 50, 80, 80]);
add_block('simulink/Commonly Used Blocks/Gain', [modelName '/D2R'], 'Gain', 'pi/180', 'Position', [120, 50, 160, 80]);

add_block('simulink/Sources/Constant', [modelName '/delta_theta_rad'], 'Position', [30, 120, 80, 150]);

add_block('simulink/Math Operations/Sum', [modelName '/Sum1'], 'Inputs', '+-', 'Position', [200, 80, 230, 110]);

add_block('simulink/Continuous/Transfer Fcn', [modelName '/PI_pitch'], ...
    'Numerator', '[1.2983]', 'Denominator', '[1 0.2968]', 'Position', [270, 80, 340, 110]);

add_block('simulink/Math Operations/Sum', [modelName '/Sum2'], 'Inputs', '+-', 'Position', [380, 80, 410, 110]);

add_block('simulink/Commonly Used Blocks/Gain', [modelName '/Neg1'], 'Gain', '-1', 'Position', [440, 80, 470, 110]);

add_block('simulink/Continuous/Transfer Fcn', [modelName '/Servo'], ...
    'Numerator', '1.2983', 'Denominator', '[1 0.2968]', 'Position', [500, 80, 570, 110]);

add_block('simulink/Commonly Used Blocks/Saturation', [modelName '/Saturation'], ...
    'UpperLimit', 'inf', 'LowerLimit', '-inf', 'Position', [600, 80, 640, 110]);

add_block('simulink/Sinks/Out1', [modelName '/delta_elevator_rad'], 'Position', [680, 80, 730, 110]);

add_block('simulink/Commonly Used Blocks/Gain', [modelName '/Gain_q'], ...
    'Gain', '0.71351', 'Position', [500, 160, 550, 190]);

add_block('simulink/Sinks/Out1', [modelName '/delta_q'], 'Position', [580, 160, 630, 190]);

% Connect blocks
add_line(modelName, 'thetacom_deg/1', 'D2R/1');
add_line(modelName, 'D2R/1', 'Sum1/1');
add_line(modelName, 'delta_theta_rad/1', 'Sum1/2');
add_line(modelName, 'Sum1/1', 'PI_pitch/1');
add_line(modelName, 'PI_pitch/1', 'Sum2/1');
add_line(modelName, 'Neg1/1', 'Sum2/2');
add_line(modelName, 'Sum2/1', 'Neg1/1');
add_line(modelName, 'Neg1/1', 'Servo/1');
add_line(modelName, 'Servo/1', 'Saturation/1');
add_line(modelName, 'Saturation/1', 'delta_elevator_rad/1');
add_line(modelName, 'Servo/1', 'Gain_q/1');
add_line(modelName, 'Gain_q/1', 'delta_q/1');

% Auto-layout for readability
Simulink.BlockDiagram.arrangeSystem(modelName);

disp(['Model "', modelName, '" created and opened.']);
