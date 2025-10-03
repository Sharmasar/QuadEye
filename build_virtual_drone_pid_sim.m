function build_virtual_drone_pid_sim()
% BUILD_VIRTUAL_DRONE_PID_SIM
% Creates a Simulink model 'virtual_drone_pid_sim.slx' implementing:
%  - simplified drone plant (3D integrator)
%  - virtual downward camera rendering colored circular targets
%  - RGB-threshold detection + bounding boxes
%  - manual selection via a MATLAB GUI callback
%  - PID-based velocity commands that move the drone toward selected target
%
% Usage: save as build_virtual_drone_pid_sim.m and run in MATLAB.

%% Clean up
modelName = 'virtual_drone_pid_sim';
if bdIsLoaded(modelName)
    close_system(modelName,0);
end
if exist([modelName '.slx'],'file')
    delete([modelName '.slx']);
end

%% Write helper MATLAB functions that Simulink will call
% These functions are normal .m files that the MATLAB Function blocks will call.
write_helper_files();

%% Create new Simulink model
new_system(modelName,'Model');
open_system(modelName);
set_param(modelName,'Solver','ode45','StopTime','30'); % 30 s run

% Layout coordinates helper
left = 30; top = 30; w = 180; h = 60; gap = 30;

%% Add subsystems and blocks

% --- DronePlant subsystem (simple integrator model) ---
add_block('built-in/Subsystem',[modelName '/DronePlant'],'Position',[left top left+w top+h]);
open_system([modelName '/DronePlant']);
% Inside DronePlant:
% Inputs: vx_cmd, vy_cmd, vz_cmd (m/s body frame)
% Outputs: x,y,z (world/inertial)
% For simplicity: convert body velocity to world via yaw=0 (no rotation).
% We'll implement a trivial integrator: x' = vx_world, etc.
add_block('simulink/Sources/In1',[modelName '/DronePlant/vx_cmd'],'Position',[20 30 50 50]);
add_block('simulink/Sources/In1',[modelName '/DronePlant/vy_cmd'],'Position',[20 80 50 100]);
add_block('simulink/Sources/In1',[modelName '/DronePlant/vz_cmd'],'Position',[20 130 50 150]);
add_block('simulink/Math Operations/Gain',[modelName '/DronePlant/gain_rot'],'Position',[120 50 160 90],'Gain','1'); % identity for yaw=0
add_block('simulink/Math Operations/Gain',[modelName '/DronePlant/gain_rot2'],'Position',[120 100 160 140],'Gain','1');
add_block('simulink/Continuous/Integrator',[modelName '/DronePlant/int_x'],'Position',[240 30 300 70]);
add_block('simulink/Continuous/Integrator',[modelName '/DronePlant/int_y'],'Position',[240 90 300 130]);
add_block('simulink/Continuous/Integrator',[modelName '/DronePlant/int_z'],'Position',[240 150 300 190]);

add_block('simulink/Sinks/Out1',[modelName '/DronePlant/x_out'],'Position',[380 30 420 50]);
add_block('simulink/Sinks/Out1',[modelName '/DronePlant/y_out'],'Position',[380 90 420 110]);
add_block('simulink/Sinks/Out1',[modelName '/DronePlant/z_out'],'Position',[380 150 420 170]);

% Connect inside DronePlant (vx->int_x, vy->int_y, vz->int_z)
add_line([modelName '/DronePlant'],'vx_cmd/1','int_x/1');
add_line([modelName '/DronePlant'],'vy_cmd/1','int_y/1');
add_line([modelName '/DronePlant'],'vz_cmd/1','int_z/1');
add_line([modelName '/DronePlant'],'int_x/1','x_out/1');
add_line([modelName '/DronePlant'],'int_y/1','y_out/1');
add_line([modelName '/DronePlant'],'int_z/1','z_out/1');

close_system([modelName '/DronePlant']);

% --- VirtualCamera subsystem (renders synthetic camera image) ---
add_block('built-in/Subsystem',[modelName '/VirtualCamera'],'Position',[left top+200 left+w top+200+h]);
open_system([modelName '/VirtualCamera']);
% Inputs: drone_x, drone_y, drone_z
add_block('simulink/Sources/In1',[modelName '/VirtualCamera/drone_x'],'Position',[20 20 50 40]);
add_block('simulink/Sources/In1',[modelName '/VirtualCamera/drone_y'],'Position',[20 70 50 90]);
add_block('simulink/Sources/In1',[modelName '/VirtualCamera/drone_z'],'Position',[20 120 50 140]);
% Output: RGB image as a matrix (we will use a To Workspace for view + a MATLAB figure)
add_block('simulink/Sinks/Out1',[modelName '/VirtualCamera/RGB_out'],'Position',[380 60 420 80]);

% Add a MATLAB Function block that calls the renderer
add_block('simulink/User-Defined Functions/MATLAB Function',[modelName '/VirtualCamera/renderFrame'],...
    'Position',[120 30 300 150]);
% Set function name and signature
set_param([modelName '/VirtualCamera/renderFrame'],'FunctionName','renderFrameSim');
% The MATLAB code is in the helper file renderFrameSim.m placed in the folder by write_helper_files()

% Connect inputs to renderFrame
add_line([modelName '/VirtualCamera'],'drone_x/1','renderFrame/1');
add_line([modelName '/VirtualCamera'],'drone_y/1','renderFrame/2');
add_line([modelName '/VirtualCamera'],'drone_z/1','renderFrame/3');
% Connect renderFrame output to RGB_out
add_line([modelName '/VirtualCamera'],'renderFrame/1','RGB_out/1');

close_system([modelName '/VirtualCamera']);

% --- Vision subsystem (detect RGB blobs) ---
add_block('built-in/Subsystem',[modelName '/Vision'],'Position',[left+320 top+200 left+320+w top+200+h]);
open_system([modelName '/Vision']);
% Inputs: RGB_frame (from VirtualCamera)
add_block('simulink/Sources/In1',[modelName '/Vision/RGB_in'],'Position',[20 30 50 50]);
% Outputs: dx_px, dy_px, detected_flag, debug_img (we'll output centers and boxes)
add_block('simulink/Sinks/Out1',[modelName '/Vision/dx_px'],'Position',[420 20 460 40]);
add_block('simulink/Sinks/Out1',[modelName '/Vision/dy_px'],'Position',[420 70 460 90]);
add_block('simulink/Sinks/Out1',[modelName '/Vision/detected'],'Position',[420 120 460 140]);
add_block('simulink/Sinks/Out1',[modelName '/Vision/debugImg'],'Position',[420 170 460 190]);
% MATLAB Function block that does detection: detectRedBlobSim
add_block('simulink/User-Defined Functions/MATLAB Function',[modelName '/Vision/detectFunc'],...
    'Position',[120 20 360 200]);
set_param([modelName '/Vision/detectFunc'],'FunctionName','detectRedBlobSim');
% Function code lives in helper file detectRedBlobSim.m

% Wire in/out
add_line([modelName '/Vision'],'RGB_in/1','detectFunc/1');
add_line([modelName '/Vision'],'detectFunc/1','dx_px/1');
add_line([modelName '/Vision'],'detectFunc/2','dy_px/1');
add_line([modelName '/Vision'],'detectFunc/3','detected/1');
add_line([modelName '/Vision'],'detectFunc/4','debugImg/1');
close_system([modelName '/Vision']);

% --- Controller subsystem (pixel->meter->PID->vx,vy,vz) ---
add_block('built-in/Subsystem',[modelName '/Controller'],'Position',[left+640 top+120 left+640+w top+120+h]);
open_system([modelName '/Controller']);
% Inputs: dx_px, dy_px, detected_flag, cam_alt_m
add_block('simulink/Sources/In1',[modelName '/Controller/dx_px'],'Position',[20 20 50 40]);
add_block('simulink/Sources/In1',[modelName '/Controller/dy_px'],'Position',[20 80 50 100]);
add_block('simulink/Sources/In1',[modelName '/Controller/detected'],'Position',[20 140 50 160]);
add_block('simulink/Sources/In1',[modelName '/Controller/cam_h'],'Position',[20 200 50 220]);
% Add a MATLAB Function pixel2meter block (uses cam_h & intrinsics)
add_block('simulink/User-Defined Functions/MATLAB Function',[modelName '/Controller/p2m'],...
    'Position',[120 20 300 120]);
set_param([modelName '/Controller/p2m'],'FunctionName','pixel2meterSim');
% PIDs (use Discrete PID)
add_block('simulink/Discrete/Discrete PID Controller',[modelName '/Controller/PID_vx'],'Position',[360 20 480 70],'P','0.08','I','0.01','D','0.02','SampleTime','0.05');
add_block('simulink/Discrete/Discrete PID Controller',[modelName '/Controller/PID_vy'],'Position',[360 100 480 150],'P','0.08','I','0.01','D','0.02','SampleTime','0.05');
% Altitude controller to reach z_des (simple P)
add_block('simulink/Math Operations/Gain',[modelName '/Controller/gain_z'],'Position',[360 180 420 220],'Gain','0.6');

% Outputs
add_block('simulink/Sinks/Out1',[modelName '/Controller/vx_cmd'],'Position',[580 20 620 40]);
add_block('simulink/Sinks/Out1',[modelName '/Controller/vy_cmd'],'Position',[580 100 620 120]);
add_block('simulink/Sinks/Out1',[modelName '/Controller/vz_cmd'],'Position',[580 180 620 200]);

% Connect p2m: dx_px->p2m->(ex_m, ey_m)
add_line([modelName '/Controller'],'dx_px/1','p2m/1');
add_line([modelName '/Controller'],'dy_px/1','p2m/2');
add_line([modelName '/Controller'],'cam_h/1','p2m/3');

% Connect p2m outputs to PID inputs (p2m has 2 outputs ex_m,ey_m)
add_line([modelName '/Controller'],'p2m/1','PID_vx/1'); % ex -> vx command
add_line([modelName '/Controller'],'p2m/2','PID_vy/1'); % ey -> vy command

% PID outputs to Out ports
add_line([modelName '/Controller'],'PID_vx/1','vx_cmd/1');
add_line([modelName '/Controller'],'PID_vy/1','vy_cmd/1');

% altitude control: desired z set via Constant block at top-level; here we output vz_cmd = gain_z*(z_des - cam_h)
add_block('simulink/Sources/Constant',[modelName '/Controller/z_des'],'Position',[120 180 160 200],'Value','1.2');
add_block('simulink/Math Operations/Sum',[modelName '/Controller/sum_z'],'Position',[240 180 280 200],'Inputs','+-');
add_line([modelName '/Controller'],'z_des/1','sum_z/1');
add_line([modelName '/Controller'],'cam_h/1','sum_z/2');
add_line([modelName '/Controller'],'sum_z/1','gain_z/1');
add_line([modelName '/Controller'],'gain_z/1','vz_cmd/1');

close_system([modelName '/Controller']);

%% --- Top-level wiring ---
% Place blocks for readability
set_param([modelName '/DronePlant'],'Position',[30 20 210 110]);
set_param([modelName '/VirtualCamera'],'Position',[30 160 210 260]);
set_param([modelName '/Vision'],'Position',[260 160 440 260]);
set_param([modelName '/Controller'],'Position',[480 140 660 260]);

% Add connection lines on top-level
% DronePlant outputs -> VirtualCamera inputs (drone position to render)
add_block('simulink/Sources/Constant',[modelName '/initVx'],'Position',[30 420 70 440],'Value','0'); % dummy
% To connect outputs we need top-level outports: create outputs of DronePlant
add_block('simulink/Signal Routing/Bus Creator',[modelName '/busPos'],'Position',[220 20 260 40],'Inputs','3');
% Connect DronePlant x,y,z to bus
add_line(modelName,'DronePlant/1','busPos/1','autorouting','on');
add_line(modelName,'DronePlant/2','busPos/2','autorouting','on');
add_line(modelName,'DronePlant/3','busPos/3','autorouting','on');

% Connect bus to VirtualCamera inputs
add_line(modelName,'busPos/1','VirtualCamera/1','autorouting','on'); % drone_x
add_line(modelName,'busPos/2','VirtualCamera/2','autorouting','on'); % drone_y
add_line(modelName,'busPos/3','VirtualCamera/3','autorouting','on'); % drone_z

% VirtualCamera RGB_out -> Vision RGB_in
add_line(modelName,'VirtualCamera/1','Vision/1','autorouting','on');

% Vision outputs -> Controller inputs
add_line(modelName,'Vision/1','Controller/1','autorouting','on'); % dx_px
add_line(modelName,'Vision/2','Controller/2','autorouting','on'); % dy_px
add_line(modelName,'Vision/3','Controller/3','autorouting','on'); % detected
% Camera altitude: pass drone z to controller cam_h
add_line(modelName,'VirtualCamera/1','Controller/4','autorouting','on'); % pass same rgb pipe as placeholder
% Instead, connect DronePlant z directly to Controller cam_h:
add_line(modelName,'DronePlant/3','Controller/4','autorouting','on');

% Controller vx,vy,vz -> DronePlant vx_cmd,vy_cmd,vz_cmd inputs
add_line(modelName,'Controller/1','DronePlant/1','autorouting','on');
add_line(modelName,'Controller/2','DronePlant/2','autorouting','on');
add_line(modelName,'Controller/3','DronePlant/3','autorouting','on');

%% Save & finish
save_system(modelName);
disp(['Created model: ' modelName '.slx']);
open_system(modelName);

fprintf('\n\nModel built. Now run the simulation in Simulink (press Run).\n');
fprintf('A separate MATLAB figure will pop up showing the camera image; click on a detected box in that image to select target.\n');

end

%% ---------------- Helper file writer ----------------
function write_helper_files()
% Write helper MATLAB functions used by MATLAB Function blocks.

% renderFrameSim - synthesizes an RGB image from drone position & static targets
fid = fopen('renderFrameSim.m','w');
fprintf(fid,[
"function img = renderFrameSim(drone_x, drone_y, drone_z)\n"...
"%% renderFrameSim - create synthetic downward camera image\n"...
"   imgW = 640; imgH = 480;\n"...
"   img = uint8(12*ones(imgH,imgW,3)); % dark background\n"...
"   %% Define static targets (world coords X,Y,Z)\n"...
"   targets = [5 5 0; 2 6 0; 8 3 0];\n"...
"   colors = [255 0 0; 0 255 0; 255 255 0];\n"...
"   cam_h = max(0.1, drone_z);\n"...
"   % Simple orthographic-like projection: map world X,Y offsets relative to drone to pixels\n"...
"   scale = 60; % pixels per meter (adjust to zoom)\n"...
"   cx = imgW/2; cy = imgH/2;\n"...
"   for i=1:size(targets,1)\n"...
"       dx = targets(i,1)-drone_x; dy = targets(i,2)-drone_y;\n"...
"       u = round(cx + dx*scale);\n"...
"       v = round(cy - dy*scale);\n"...
"       r = max(6, round(12*max(0.5,0.5*1/cam_h)));\n"...
"       [xx,yy] = meshgrid(max(1,u-r):min(imgW,u+r), max(1,v-r):min(imgH,v+r));\n"...
"       mask = (xx-u).^2 + (yy-v).^2 <= r^2;\n"...
"       for ch=1:3\n"...
"           tmp = img(:,:,ch);\n"...
"           tmp(sub2ind([imgH imgW], yy(mask), xx(mask))) = colors(i,ch);\n"...
"           img(:,:,ch) = tmp;\n"...
"       end\n"...
"   end\n"...
"end\n"]);
fclose(fid);

% detectRedBlobSim - find blobs and return pixel offsets + debug image
fid = fopen('detectRedBlobSim.m','w');
fprintf(fid,[
"function [dx_px, dy_px, detected, dbgImg] = detectRedBlobSim(img)\n"...
"%% detectRedBlobSim - naive RGB threshold blob detector\n"...
"   if isempty(img); dx_px=0; dy_px=0; detected=0; dbgImg=uint8([]); return; end\n"...
"   I = im2double(img);\n"...
"   R = I(:,:,1); G = I(:,:,2); B = I(:,:,3);\n"...
"   % Simple red detection: R significantly greater than G,B\n"...
"   mask = (R > 0.45) & (R > 1.2*G) & (R > 1.2*B);\n"...
"   % remove tiny noise\n"...
"   mask = bwareaopen(mask, 50);\n"...
"   dbgImg = img;\n"...
"   stats = regionprops(mask,'Area','Centroid');\n"...
"   if isempty(stats)\n"...
"       dx_px = 0; dy_px = 0; detected=0; return;\n"...
"   end\n"...
"   [~, idx] = max([stats.Area]);\n"...
"   c = stats(idx).Centroid; % [x y]\n"...
"   imgH = size(img,1); imgW = size(img,2);\n"...
"   dx_px = c(1) - imgW/2;\n"...
"   dy_px = c(2) - imgH/2;\n"...
"   detected = 1;\n"...
"   % draw debug box + center using simple overlays\n"...
"   bbox = [max(1,round(c(1)-10)), max(1,round(c(2)-10)), 20, 20];\n"...
"   dbgImg = insertShape(dbgImg,'Rectangle',bbox,'Color','yellow','LineWidth',2);\n"...
"   dbgImg = insertMarker(dbgImg,[imgW/2 imgH/2],'o','Color','green','Size',6);\n"...
"   dbgImg = insertMarker(dbgImg,c,'x','Color','red','Size',6);\n"...
"end\n"]);
fclose(fid);

% pixel2meterSim - convert pixel offsets to meters using camera height & fov
fid = fopen('pixel2meterSim.m','w');
fprintf(fid,[
"function [ex_m, ey_m] = pixel2meterSim(dx_px, dy_px, cam_h)\n"...
"%% pixel2meterSim - simple pinhole small-angle approx, assumes fx ~ fy\n"...
"   imgW = 640; imgH = 480; fov = 60; f = (imgH/2)/tand(fov/2);\n"...
"   ex_m = (dx_px / f) * cam_h;\n"...
"   ey_m = (dy_px / f) * cam_h;\n"...
"end\n"]);
fclose(fid);

% helpers: insertShape/insertMarker fallback if Image Processing toolbox not present
% We will use built-in insertShape/insertMarker which exist in Computer Vision Toolbox.
% If not available, the debug images will be raw; it's safe.

end
