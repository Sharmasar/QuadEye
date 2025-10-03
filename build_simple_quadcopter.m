function build_simple_quadcopter()
    model = 'simple_quadcopter_pid';
    new_system(model);
    open_system(model);

    % ------------------------
    % Add main subsystems
    % ------------------------
    % Quadcopter plant (placeholder transfer function / dynamics block)
    add_block('simulink/Continuous/Transfer Fcn', [model '/QuadcopterPlant'], ...
        'Numerator', '1', 'Denominator', '[1 2 1]', ...
        'Position', [350 150 430 180]);

    % PID Controller
    add_block('simulink/Continuous/PID Controller', [model '/PID'], ...
        'P', '2', 'I', '0.5', 'D', '0.1', ...
        'Position', [150 140 230 180]);

    % Reference (step input)
    add_block('simulink/Sources/Step', [model '/Reference'], ...
        'Time', '0', 'Before', '0', 'After', '1', ...
        'Position', [50 150 80 180]);

    % Sum block
    add_block('simulink/Math Operations/Sum', [model '/Sum'], ...
        'Inputs', '+-', 'Position', [250 150 270 180]);

    % Scope
    add_block('simulink/Sinks/Scope', [model '/Scope'], ...
        'Position', [500 150 530 180]);

    % ------------------------
    % Wire blocks together
    % ------------------------
    add_line(model, 'Reference/1', 'Sum/1');
    add_line(model, 'QuadcopterPlant/1', 'Scope/1');
    add_line(model, 'PID/1', 'QuadcopterPlant/1');
    add_line(model, 'Sum/1', 'PID/1');
    add_line(model, 'QuadcopterPlant/1', 'Sum/2');

    % Save model
    save_system(model);
    fprintf('✅ Model "%s.slx" created successfully.\n', model);
end
