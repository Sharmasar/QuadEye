function create_integrated_quadcopter_control()
% CREATE_INTEGRATED_QUADCOPTER_CONTROL
% Creates a Simulink model "integrated_quadcopter_control.slx" that:
% - tries to include user's downloaded quadcopter model if found (searches current folder tree)
% - or else uses a fallback simplified plant
% - adds a vision -> pixel->meter -> PID -> command pipeline
% - saves the model and prints manual wiring instructions if needed.
%
% Run from the folder containing your downloaded quadcopter SLX (or any parent folder).

%% Config
outModel = 'integrated_quadcopter_control';
simStopTime = '60'; % seconds, string

% Searching for user quadcopter model (slx or mdl files with 'quadcopter' in name)
fprintf('Searching for quadcopter model files under "%s"...\n', pwd);
slxFiles = dir(fullfile(pwd, '**', '*quadcopter*.slx'));
mdlFiles = dir(fullfile(pwd, '**', '*quadcopter*.mdl'));
foundModel = '';
if ~isempty(slxFiles)
    % prefer the first .slx
    foundModel = fullfile(slxFiles(1).folder, slxFiles(1).name);
elseif ~isempty(mdlFiles)
    foundModel = fullfile(mdlFiles(1).folder, mdlFiles(1).name);
end

%% Create new model (overwrite if exists)
if bdIsLoaded(outModel)
    close_system(outModel,0);
end
if exist([outModel '.slx'],'file')
    delete([outModel '.slx']);
end
new_system(outModel);
open_system(outModel);
set_param(outModel,'Solver','ode45','StopTime',simStopTime);

% Layout params
x0 = 30; y0 = 30; w = 230; h = 80; vGap = 130;

%% 1) Add Vision + Controller subsystem (pixel→meter→PID)
visionCtrlPos = [x0 y0 x0+w y0+h];
add_block('built-in/Subsystem',[outModel '/Vision_and_Controller'],'Position',visionCtrlPos);
open_system([outModel '/Vision_and_Controller']);

% Inside Vision_and_Controller: In ports, Out ports
% Inputs: RGB image (as MxNx3 uint8), cam_height_m
% Outputs: vx_cmd, vy_cmd, vz_cmd  (m/s)
add_block('simulink/Sources/In1',[outModel '/Vision_and_Controller/RGB_in'],'Position',[20 20 50 40]);
add_block('simulink/Sources/In1',[outModel '/Vision_and_Controller/cam_h_m'],'Position',[20 80 50 100]);

% MATLAB Function: detect + pixel->meter
mfDetectPath = [outModel '/Vision_and_Controller/detect_pixel2meter'];
add_block('simulink/User-Defined Functions/MATLAB Function', mfDetectPath, 'Position',[120 20 360 140]);
% Set its FunctionName and rely on an external file to implement it (safer)
% We'll write the helper file detect_and_project_sim.m into pwd:
write_detect_and_project_helper();

% Configure MATLAB Function to call the helper (set function name)
set_param(mfDetectPath,'FunctionName','detect_and_project_sim');

% Add PID controllers (discrete for simplicity)
add_block('simulink/Discrete/Discrete PID Controller',[outModel '/Vision_and_Controller/PID_vx'],...
    'P','0.5','I','0.05','D','0.01','SampleTime','0.02','Position',[400 20 520 70]);
add_block('simulink/Discrete/Discrete PID Controller',[outModel '/Vision_and_Controller/PID_vy'],...
    'P','0.5','I','0.05','D','0.01','SampleTime','0.02','Position',[400 90 520 140]);
add_block('simulink/Math Operations/Gain',[outModel '/Vision_and_Controller/gain_vz'],...
    'Gain','0.6','Position',[400 200 460 240]);

% Outports
add_block('simulink/Sinks/Out1',[outModel '/Vision_and_Controller/vx_cmd'],'Position',[640 30 680 50]);
add_block('simulink/Sinks/Out1',[outModel '/Vision_and_Controller/vy_cmd'],'Position',[640 100 680 120]);
add_block('simulink/Sinks/Out1',[outModel '/Vision_and_Controller/vz_cmd'],'Position',[640 200 660 220]);

% Connect inside Vision_and_Controller:
% RGB_in -> detect_pixel2meter (arg1)
add_line([outModel '/Vision_and_Controller'],'RGB_in/1','detect_pixel2meter/1','autorouting','on');
% cam_h_m -> detect_pixel2meter arg2
add_line([outModel '/Vision_and_Controller'],'cam_h_m/1','detect_pixel2meter/2','autorouting','on');

% detect outputs -> PID inputs:
% detect_and_project_sim returns [ex_m, ey_m, detected, dbgImg]
add_line([outModel '/Vision_and_Controller'],'detect_pixel2meter/1','PID_vx/1','autorouting','on'); % ex_m
add_line([outModel '/Vision_and_Controller'],'detect_pixel2meter/2','PID_vy/1','autorouting','on'); % ey_m

% PID outputs -> outports
add_line([outModel '/Vision_and_Controller'],'PID_vx/1','vx_cmd/1','autorouting','on');
add_line([outModel '/Vision_and_Controller'],'PID_vy/1','vy_cmd/1','autorouting','on');

% vz: simple altitude error as example: desired_z - cam_h
% add a desired z constant inside subsystem and compute vz_cmd = gain*(z_des - cam_h)
add_block('simulink/Sources/Constant',[outModel '/Vision_and_Controller/z_des'],'Position',[120 200 160 220],'Value','1.2'); % target altitude 1.2 m
add_block('simulink/Math Operations/Sum',[outModel '/Vision_and_Controller/sum_z'],'Position',[220 200 260 220],'Inputs','+-');
add_line([outModel '/Vision_and_Controller'],'z_des/1','sum_z/1','autorouting','on');
add_line([outModel '/Vision_and_Controller'],'cam_h_m/1','sum_z/2','autorouting','on');
add_line([outModel '/Vision_and_Controller'],'sum_z/1','gain_vz/1','autorouting','on');
add_line([outModel '/Vision_and_Controller'],'gain_vz/1','vz_cmd/1','autorouting','on');

close_system([outModel '/Vision_and_Controller']);

%% 2) Add Plant (either user's quadcopter model or fallback)
plantPos = [x0 y0+vGap x0+w y0+vGap+h];
if ~isempty(foundModel)
    fprintf('Found quadcopter model: %s\n', foundModel);
    [~,name,ext] = fileparts(foundModel);
    % Load that model (if not loaded)
    try
        load_system(foundModel);
    catch
        warning('Failed to load model file automatically: %s', foundModel);
    end
    % Add a Model block referencing that model
    add_block('simulink/Ports & Subsystems/Model',[outModel '/User_Quadcopter'],'Position',plantPos);
    try
        set_param([outModel '/User_Quadcopter'],'ModelName',name);
        fprintf('Added model reference block to "%s" (ModelName=%s).\n', outModel, name);
        usedUserModel = true;
    catch ME
        warning('Could not set ModelName parameter: %s\nWill instead create a simple fallback plant.', ME.message);
        delete_block([outModel '/User_Quadcopter']);
        usedUserModel = false;
    end
else
    fprintf('No quadcopter model found. Using fallback plant.\n');
    usedUserModel = false;
end

% If not using user model, create fallback plant (simple integrators)
if ~usedUserModel
    add_block('simulink/Continuous/Integrator',[outModel '/Plant/int_x'],'Position',[x0+420 y0+vGap+20 x0+470 y0+vGap+60]);
    add_block('simulink/Continuous/Integrator',[outModel '/Plant/int_y'],'Position',[x0+420 y0+vGap+100 x0+470 y0+vGap+140]);
    add_block('simulink/Continuous/Integrator',[outModel '/Plant/int_z'],'Position',[x0+420 y0+vGap+180 x0+470 y0+vGap+220]);
    add_block('simulink/Sinks/Out1',[outModel '/Plant/x_out'],'Position',[x0+520 y0+vGap+20 x0+560 y0+vGap+40]);
    add_block('simulink/Sinks/Out1',[outModel '/Plant/y_out'],'Position',[x0+520 y0+vGap+100 x0+560 y0+vGap+120]);
    add_block('simulink/Sinks/Out1',[outModel '/Plant/z_out'],'Position',[x0+520 y0+vGap+180 x0+560 y0+vGap+200]);
    % Input ports (vx,vy,vz)
    add_block('simulink/Sources/In1',[outModel '/Plant/vx_in'],'Position',[x0+360 y0+vGap+20 x0+390 y0+vGap+40]);
    add_block('simulink/Sources/In1',[outModel '/Plant/vy_in'],'Position',[x0+360 y0+vGap+100 x0+390 y0+vGap+120]);
    add_block('simulink/Sources/In1',[outModel '/Plant/vz_in'],'Position',[x0+360 y0+vGap+180 x0+390 y0+vGap+200]);
    add_line(outModel,'Plant/vx_in/1','Plant/int_x/1','autorouting','on');
    add_line(outModel,'Plant/vy_in/1','Plant/int_y/1','autorouting','on');
    add_line(outModel,'Plant/vz_in/1','Plant/int_z/1','autorouting','on');
    add_line(outModel,'Plant/int_x/1','Plant/x_out/1','autorouting','on');
    add_line(outModel,'Plant/int_y/1','Plant/y_out/1','autorouting','on');
    add_line(outModel,'Plant/int_z/1','Plant/z_out/1','autorouting','on');
    fprintf('Fallback plant (integrators) created inside model.\n');
end

%% 3) Top-level wiring: connect Vision_and_Controller outputs to Plant inputs
% Vision_and_Controller is at position set earlier; find its port numbers
% At top-level the Vision_and_Controller has outports vx_cmd(1), vy_cmd(2), vz_cmd(3)
% For user model we cannot guarantee port names; we try to connect automatically to common port names.

% Make the top-level connections for fallback case
if ~usedUserModel
    % Connect Vision vx->Plant vx_in, vy->vy_in, vz->vz_in
    add_line(outModel,'Vision_and_Controller/1','Plant/vx_in/1','autorouting','on'); % vx
    add_line(outModel,'Vision_and_Controller/2','Plant/vy_in/1','autorouting','on'); % vy
    add_line(outModel,'Vision_and_Controller/3','Plant/vz_in/1','autorouting','on'); % vz
    % Connect Plant outputs to VirtualCamera inputs (position -> camera)
    add_line(outModel,'Plant/x_out/1','VirtualCamera/1','autorouting','on');
    add_line(outModel,'Plant/y_out/1','VirtualCamera/2','autorouting','on');
    add_line(outModel,'Plant/z_out/1','VirtualCamera/3','autorouting','on');
    % Connect VirtualCamera RGB_out to Vision RGB_in
    add_line(outModel,'VirtualCamera/1','Vision/1','autorouting','on');
    % Connect camera height (z) to Vision_and_Controller cam_h_m (we'll take Plant z_out)
    add_line(outModel,'Plant/z_out/1','Vision_and_Controller/2','autorouting','on');
else
    % If using user model, try to detect typical outputs/inputs by inspecting model ports.
    try
        userModelName = get_param(foundModel,'Name');
    catch
        [~,userModelName] = fileparts(foundModel);
    end
    fprintf('\nAttempting to auto-wire to user model: %s\n', userModelName);
    fprintf('NOTE: automatic wiring may fail if the model top-level ports do not match expected names.\n');
    % Add comment block to instruct user if needed
    blkComment = add_block('simulink/Ports & Subsystems/Subsystem',...
        [outModel '/USER_MODEL_WIRING_NOTE'],'Position',[900 40 1200 140]);
    set_param([outModel '/USER_MODEL_WIRING_NOTE'],'Mask','on');
    set_param([outModel '/USER_MODEL_WIRING_NOTE'],'BackgroundColor','yellow');
    % Provide instructions in model annotation
    annotationText = sprintf(['AUTO-INTEGRATION:\nModel referenced: %s\n\nIf automatic wiring failed, open the model and connect the Controller outputs (vx,vy,vz)\nto the corresponding input ports of your quadcopter model (e.g. velocity or motor commands).\nAlso connect the model outputs for x,y,z back to VirtualCamera inputs so the camera knows\nwhere the drone is.\n\nIf you want, paste here the top-level ports of your quadcopter model and I will update the script to wire them.'], userModelName);
    annotation = add_block('simulink/Annotations/Note',[outModel '/note1'],'Position',[900 160 1200 300]);
    set_param([outModel '/note1'],'Text',annotationText);
    % We leave the manual wiring for the user when using their model
    fprintf('\nManual steps (if required):\n');
    fprintf('1) Open "%s" (your model). Identify the input ports that accept commanded vx,vy,vz or attitude/thrust.\n', foundModel);
    fprintf('2) Connect Vision_and_Controller outputs (vx_cmd, vy_cmd, vz_cmd) to those input ports.\n');
    fprintf('3) Connect model position outputs (x,y,z) back to VirtualCamera inputs (so virtual camera knows drone pose).\n\n');
end

%% 4) Final touches: save and open
save_system(outModel);
fprintf('\nCreated model: %s.slx\n', outModel);
open_system(outModel);

fprintf('\nFINISHED. Next steps:\n');
if usedUserModel
    fprintf(' - Open the model and check that Vision_and_Controller outputs are connected to your quadcopter model inputs.\n');
    fprintf(' - If connection fails, follow the manual wiring instructions printed above.\n');
else
    fprintf(' - Model is fully wired with a fallback plant. Run the simulation and watch the logs.\n');
end
fprintf('\nRun simulation from Simulink (press Run). If anything errors, copy the exact error text here and I will fix the script.\n');

end

%% ---------- Helper that writes detect_and_project_sim.m ----------
function write_detect_and_project_helper()
% Writes a compact helper file detect_and_project_sim.m used by the MATLAB Function in
% Vision_and_Controller -> detect_and_project_sim(dx_px,...)

fid = fopen('detect_and_project_sim.m','w');
if fid == -1
    error('Could not create detect_and_project_sim.m in current folder. Check write permissions.');
end

fprintf(fid, [
"function [ex_m, ey_m, detected, dbgImg] = detect_and_project_sim(img, cam_h_m)\n"...
"%% Simple detect + pixel->meter helper for Simulink\n"...
"% img: uint8 RGB image (HxWx3)\n"...
"% cam_h_m: camera altitude in meters\n"% newline...
"if isempty(img) || (size(img,1)<2)\n"...
"    ex_m=0; ey_m=0; detected=0; dbgImg=uint8(zeros(480,640,3)); return; end\n"...
"img = im2uint8(img);\n"...
"% naive red detection (tune thresholds if needed)\n"I = im2double(img);\n"...
"R = I(:,:,1); G = I(:,:,2); B = I(:,:,3);\n"...
"mask = (R > 0.5) & (R > 1.2*G) & (R > 1.2*B);\n"...
"mask = bwareaopen(mask, 30);\n"...
"stats = regionprops(mask, 'Area', 'Centroid');\n"...
"dbgImg = img;\n"...
"if isempty(stats)\n"...
"    ex_m = 0; ey_m = 0; detected = 0; return;\n"...
"end\n"...
"[~, idx] = max([stats.Area]); c = stats(idx).Centroid; imgH = size(img,1); imgW = size(img,2);\n"...
"% pixel error\n"...
"dx_px = c(1) - imgW/2; dy_px = c(2) - imgH/2;\n"...
"% convert to meters using pinhole model (fov approx 60 deg)\n"...
"fov = 60; f = (imgH/2)/tand(fov/2);\n"...
"ex_m = (dx_px / f) * cam_h_m; ey_m = (dy_px / f) * cam_h_m;\n"...
"detected = 1;\n"...
"% draw debug (if Computer Vision Toolbox available, insertShape/insertMarker will work)\n"...
"try\n"...
"    bbox = [max(1,round(c(1)-10)), max(1,round(c(2)-10)), 20, 20];\n"...
"    dbgImg = insertShape(dbgImg,'Rectangle',bbox,'Color','yellow','LineWidth',2);\n"...
"    dbgImg = insertMarker(dbgImg,[imgW/2 imgH/2],'o','Color','green','Size',6);\n"...
"    dbgImg = insertMarker(dbgImg,c,'x','Color','red','Size',6);\n"...
"catch\n"...
"    % If insertShape not available, skip debug overlays\n"...
"end\n"...
"end\n"]);
fclose(fid);
fprintf('Wrote helper: detect_and_project_sim.m\n');
end
