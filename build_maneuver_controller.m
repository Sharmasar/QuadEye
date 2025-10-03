function build_maneuver_controller()
    % Create a new model
    modelName = 'ManeuverControllerModel';
    new_system(modelName);
    open_system(modelName);

    % Add a subsystem for the Maneuver Controller
    add_block('built-in/Subsystem', [modelName '/ManeuverController'], ...
        'Position', [100 100 300 250]);

    % Go inside the subsystem
    open_system([modelName '/ManeuverController']);

    % Add MATLAB Function block (Carrot Chasing Guidance Law)
    mfBlock = add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [modelName '/ManeuverController/CarrotChaser'], ...
        'Position', [100 100 250 180]);
    
    % Define MATLAB Function code
    code = [
        "function [x_cmd, y_cmd, z_cmd, yaw_cmd] = CarrotChaser(curr_x, curr_y, curr_z, waypoints)" newline ...
        "%#codegen" newline ...
        "    % Simple carrot chasing: pick next waypoint as target" newline ...
        "    target = waypoints(:,end);" newline ...
        "    x_cmd = target(1);" newline ...
        "    y_cmd = target(2);" newline ...
        "    z_cmd = target(3);" newline ...
        "    yaw_cmd = atan2(target(2)-curr_y, target(1)-curr_x);" newline ...
        "end"
    ];
    set_param(mfBlock, 'MATLABFunction', code);

    % Add PID controllers
    add_block('simulink/Continuous/PID Controller', ...
        [modelName '/ManeuverController/PID_Roll'], 'Position', [400 50 500 100]);
    add_block('simulink/Continuous/PID Controller', ...
        [modelName '/ManeuverController/PID_Pitch'], 'Position', [400 120 500 170]);
    add_block('simulink/Continuous/PID Controller', ...
        [modelName '/ManeuverController/PID_Yaw'], 'Position', [400 190 500 240]);
    add_block('simulink/Continuous/PID Controller', ...
        [modelName '/ManeuverController/PID_Altitude'], 'Position', [400 260 500 310]);

    % Add input and output ports
    add_block('simulink/Sources/In1', [modelName '/ManeuverController/curr_x'], 'Position', [20 60 50 80]);
    add_block('simulink/Sources/In1', [modelName '/ManeuverController/curr_y'], 'Position', [20 100 50 120]);
    add_block('simulink/Sources/In1', [modelName '/ManeuverController/curr_z'], 'Position', [20 140 50 160]);
    add_block('simulink/Sources/In1', [modelName '/ManeuverController/waypoints'], 'Position', [20 180 50 200]);

    add_block('simulink/Sinks/Out1', [modelName '/ManeuverController/roll_cmd'], 'Position', [600 70 630 90]);
    add_block('simulink/Sinks/Out1', [modelName '/ManeuverController/pitch_cmd'], 'Position', [600 140 630 160]);
    add_block('simulink/Sinks/Out1', [modelName '/ManeuverController/yaw_cmd'], 'Position', [600 210 630 230]);
    add_block('simulink/Sinks/Out1', [modelName '/ManeuverController/throttle_cmd'], 'Position', [600 280 630 300]);

    % Save the model
    save_system(modelName, [modelName '.slx']);
    disp(['Model saved as ' modelName '.slx']);
end
