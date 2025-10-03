function ARCsim
%% UAV + Downward Virtual Camera + Object Detection + PID (Simulink, no Unreal)
% Frames:
%   - World inertial ENU: x=East, y=North, z=Up
%   - Camera: x right, y down, z forward. Mounted so z_forward = -world_Z (down-looking)
%
% Requires: Simulink, Computer Vision Toolbox, Control System Toolbox

clc; close all;

%% ---------------- Parameters (base workspace) ----------------
Ts        = 0.02;          % s, fixed step
simTime   = 25;            % s

% Drone initial state in ENU
dronePos0 = [0; 0; 6];     % start 6 m above ground
droneVel0 = [0; 0; 0];

% Targets (world ENU positions). You can add more rows.
targetsENU = [ 5 5 0;
               2 7 0;
               7 2 0 ];

targetRadius = 0.35;       % m (physical radius used for rendering)

% Desired constant altitude (z Up)
desiredAltitude = 3.0;     % m

% Camera intrinsics (pinhole, square pixels)
imgW = 960; imgH = 720;    % bigger feed so it looks nice
fovV_deg = 60;             % vertical FOV
fy = (imgH/2) / tand(fovV_deg/2);
fx = fy; cx = imgW/2; cy = imgH/2;

% Camera mounting: cv -> world
% Make camera optical axis (+z_cv) point DOWN (−world Z) when level
R_world_from_cv = [ 1,  0,  0;
                    0,  1,  0;
                    0,  0, -1 ];

% PID gains
Kp_pix = 0.05; Ki_pix = 0.0;  Kd_pix = 0.02;   % image (pixels) → vx, vy
Kp_alt = 1.0;  Ki_alt = 0.2;  Kd_alt = 0.2;    % altitude (m)    → vz

vmaxENU = [2; 2; 1.5];      % m/s saturation

% Put in base for Simulink
assignin('base',"Ts",Ts);
assignin('base',"simTime",simTime);
assignin('base',"dronePos0",dronePos0);
assignin('base',"droneVel0",droneVel0);
assignin('base',"targetsENU",targetsENU);
assignin('base',"targetRadius",targetRadius);
assignin('base',"desiredAltitude",desiredAltitude);
assignin('base',"imgW",imgW);
assignin('base',"imgH",imgH);
assignin('base',"fx",fx);
assignin('base',"fy",fy);
assignin('base',"cx",cx);
assignin('base',"cy",cy);
assignin('base',"R_world_from_cv",R_world_from_cv);
assignin('base',"Kp_pix",Kp_pix);
assignin('base',"Ki_pix",Ki_pix);
assignin('base',"Kd_pix",Kd_pix);
assignin('base',"Kp_alt",Kp_alt);
assignin('base',"Ki_alt",Ki_alt);
assignin('base',"Kd_alt",Kd_alt);
assignin('base',"vmaxENU",vmaxENU);

%% ---------------- Preview: render + detect + click selection ----------------
[previewFrame, uvAll] = renderVirtualCameraFrame(dronePos0, targetsENU, ...
    fx,fy,cx,cy,imgW,imgH, R_world_from_cv, targetRadius);

detBBoxes = detectRedBlobs(previewFrame);   % simple detector that works on our render

[selectedTargetIdx] = uiSelectTarget(previewFrame, detBBoxes, uvAll);
if isempty(selectedTargetIdx)
    disp('No selection made; defaulting to first target.');
    selectedTargetIdx = 1;
end
assignin('base',"selectedTargetIdx",selectedTargetIdx);

%% ---------------- Build Simulink model ----------------
model = "UAV_VirtCam_Sim";
if bdIsLoaded(model), close_system(model,0); end
new_system(model); open_system(model);
set_param(model,'StopTime',num2str(simTime));
set_param(model,'Solver','discrete (no continuous states)', ...
                 'FixedStep',num2str(Ts), ...
                 'SolverType','Fixed-step');

% Layout helper
x0=30; y0=30; DX=170; DY=100;
pos = @(r,c,w,h) [x0+c*DX, y0+r*DY, x0+c*DX+w, y0+r*DY+h];

% Sources
add_block('simulink/Sources/Constant', model+"/Targets",     'Value','targetsENU',    'Position',pos(0,0,120,40));
add_block('simulink/Sources/Constant', model+"/SelIdx",      'Value','selectedTargetIdx','Position',pos(1,0,120,40));
add_block('simulink/Sources/Constant', model+"/DesiredAlt",  'Value','desiredAltitude','Position',pos(2,0,120,40));
add_block('simulink/Sources/Constant', model+"/fx",'Value','fx','Position',pos(0,1,60,30));
add_block('simulink/Sources/Constant', model+"/fy",'Value','fy','Position',pos(1,1,60,30));
add_block('simulink/Sources/Constant', model+"/cx",'Value','cx','Position',pos(2,1,60,30));
add_block('simulink/Sources/Constant', model+"/cy",'Value','cy','Position',pos(3,1,60,30));
add_block('simulink/Sources/Constant', model+"/imgW",'Value','imgW','Position',pos(4,1,80,30));
add_block('simulink/Sources/Constant', model+"/imgH",'Value','imgH','Position',pos(5,1,80,30));

% Discrete integrators for velocity & position (ENU)
add_block('simulink/Discrete/Discrete-Time Integrator', model+"/VEL INT", ...
    'gainval','Ts','InitialCondition','droneVel0','Position',pos(0,2,120,50));
add_block('simulink/Discrete/Discrete-Time Integrator', model+"/POS INT", ...
    'gainval','Ts','InitialCondition','dronePos0','Position',pos(1,2,120,50));
add_line(model,"VEL INT/1","POS INT/1",'autorouting',"on");

% Bus for state (pos,vel)
add_block('simulink/Signal Routing/Bus Creator', model+"/StateBus",'Inputs','2','Position',pos(2,2,80,50));
add_line(model,"POS INT/1","StateBus/1",'autorouting',"on");
add_line(model,"VEL INT/1","StateBus/2",'autorouting',"on");

% Bus selector to grab pos for camera
add_block('simulink/Signal Routing/Bus Selector', model+"/StateSel",'OutputSignals','1,2','Position',pos(3,2,80,60));
add_line(model,"StateBus/1","StateSel/1",'autorouting',"on");

% MATLAB Function: CameraModel
add_block('simulink/User-Defined Functions/MATLAB Function', model+"/CameraModel", 'Position',pos(0,3,270,120));
set_param(model+"/CameraModel",'Script', cameraModelFcn);
% Wire inputs: pos, targets, fx,fy,cx,cy,imgW,imgH
add_line(model,"StateSel/1","CameraModel/1",'autorouting',"on");
add_line(model,"Targets/1","CameraModel/2",'autorouting',"on");
add_line(model,"fx/1","CameraModel/3",'autorouting',"on");
add_line(model,"fy/1","CameraModel/4",'autorouting',"on");
add_line(model,"cx/1","CameraModel/5",'autorouting',"on");
add_line(model,"cy/1","CameraModel/6",'autorouting',"on");
add_line(model,"imgW/1","CameraModel/7",'autorouting',"on");
add_line(model,"imgH/1","CameraModel/8",'autorouting',"on");

% Video Viewer to display virtual camera
add_block('vision/Video Viewer', model+"/Video Viewer", 'Position',pos(0,4,150,60));
add_line(model,"CameraModel/1","Video Viewer/1",'autorouting',"on");

% MATLAB Function: PixelError (uses uvAll + selection → u,v error in pixels)
add_block('simulink/User-Defined Functions/MATLAB Function', model+"/PixelError", 'Position',pos(1,3,190,110));
set_param(model+"/PixelError",'Script', pixelErrorFcn);
add_line(model,"CameraModel/2","PixelError/1",'autorouting',"on");
add_line(model,"SelIdx/1","PixelError/2",'autorouting',"on");
add_line(model,"cx/1","PixelError/3",'autorouting',"on");
add_line(model,"cy/1","PixelError/4",'autorouting',"on");

% PID (u, v) → vx, vy; Altitude PID → vz
add_block('simulink/Discrete/Discrete PID Controller', model+"/PID_u", ...
    'P','Kp_pix','I','Ki_pix','D','Kd_pix','SampleTime','Ts','Form','Parallel', ...
    'Position',pos(1,4,130,60));
add_block('simulink/Discrete/Discrete PID Controller', model+"/PID_v", ...
    'P','Kp_pix','I','Ki_pix','D','Kd_pix','SampleTime','Ts','Form','Parallel', ...
    'Position',pos(2,4,130,60));
add_block('simulink/Discrete/Discrete PID Controller', model+"/PID_z", ...
    'P','Kp_alt','I','Ki_alt','D','Kd_alt','SampleTime','Ts','Form','Parallel', ...
    'Position',pos(3,4,130,60));

% Negate PID outputs (command drives error toward zero)
add_block('simulink/Math Operations/Gain', model+"/NegU", 'Gain','-1','Position',pos(1,5,60,40));
add_block('simulink/Math Operations/Gain', model+"/NegV", 'Gain','-1','Position',pos(2,5,60,40));
add_block('simulink/Math Operations/Gain', model+"/NegZ", 'Gain','-1','Position',pos(3,5,60,40));

add_line(model,"PixelError/1","PID_u/1",'autorouting',"on"); % uErr
add_line(model,"PixelError/2","PID_v/1",'autorouting',"on"); % vErr

% Altitude error = desiredAltitude - z
add_block('simulink/Signal Routing/Selector', model+"/SelZ", ...
    'IndexOptionArray','Index vector (dialog)', ...
    'IndexParamArray','3', 'NumberOfDimensions','1', ...
    'InputPortWidth','3', 'Position', pos(4,2,80,40));
add_block('simulink/Math Operations/Sum', model+"/AltErr", 'Inputs','+-', 'Position', pos(3,3,70,40));
add_line(model,"StateSel/1","SelZ/1",'autorouting',"on");        % pos
add_line(model,"DesiredAlt/1","AltErr/1",'autorouting',"on");
add_line(model,"SelZ/1","AltErr/2",'autorouting',"on");
add_line(model,"AltErr/1","PID_z/1",'autorouting',"on");

add_line(model,"PID_u/1","NegU/1",'autorouting',"on");
add_line(model,"PID_v/1","NegV/1",'autorouting',"on");
add_line(model,"PID_z/1","NegZ/1",'autorouting',"on");

% Mux velocity command and saturate
add_block('simulink/Signal Routing/Mux', model+"/MuxVel", 'Inputs','3','Position',pos(4,5,60,60));
add_line(model,"NegU/1","MuxVel/1",'autorouting',"on"); % vx (East)
add_line(model,"NegV/1","MuxVel/2",'autorouting',"on"); % vy (North)
add_line(model,"NegZ/1","MuxVel/3",'autorouting',"on"); % vz (Up)

add_block('simulink/Discontinuities/Saturation', model+"/SatVel", ...
    'UpperLimit','vmaxENU''','LowerLimit','-vmaxENU''', 'Position', pos(5,5,100,50));
add_line(model,"MuxVel/1","SatVel/1",'autorouting',"on");

% Send to velocity integrator
add_line(model,"SatVel/1","VEL INT/1",'autorouting',"on");

% Scopes to watch commands (optional)
add_block('simulink/Sinks/Scope', model+"/Cmd Scope", 'Position', pos(5,3,120,80));
add_line(model,"NegU/1","Cmd Scope/1",'autorouting',"on");
add_line(model,"NegV/1","Cmd Scope/2",'autorouting',"on");
add_line(model,"NegZ/1","Cmd Scope/3",'autorouting',"on");

% Run
set_param(model,'SimulationCommand','start');

%% ---------------- Local helper functions ----------------
function code = cameraModelFcn
% MATLAB Function block: render downward camera frame + per-target pixel centers
code = [
"function [frame, uvAll] = fcn(posENU, targetsENU, fx,fy,cx,cy,imgW,imgH)"
"%#codegen"
"% posENU      : [3x1] double"
"% targetsENU  : [N x 3] double"
"% Outputs:"
"%   frame  [imgH x imgW x 3] uint8  - synthetic camera image"
"%   uvAll  [N x 2] double           - pixel centers for each target"
""
"R_world_from_cv = evalin('base','R_world_from_cv');"
"targetRadius    = evalin('base','targetRadius');"
""
"imgW = double(imgW); imgH = double(imgH);"
"fx = double(fx); fy = double(fy); cx = double(cx); cy = double(cy);"
""
"% World->camera rotation"
"R_cv_from_world = R_world_from_cv';"
"cam_w = double(posENU(:)');   % camera position in world"
"N = size(targetsENU,1);"
"uvAll = nan(N,2);"
""
"% Start with dark gray background so it's not black"
"frame = uint8(ones(imgH,imgW,3)*40);"
""
"% Draw image center cross (green)"
"ccx = round(cx); ccy = round(cy);"
"if ccx>=1 && ccx<=imgW && ccy>=1 && ccy<=imgH"
"    frame(ccy, max(1,ccx-5):min(imgW,ccx+5), 2) = 255;"
"    frame(max(1,ccy-5):min(imgH,ccy+5), ccx, 2) = 255;"
"end"
""
"% Render each target as a filled red disk using pinhole"
"for i=1:N"
"    pw = double(targetsENU(i,1:3));"
"    pcv = (R_cv_from_world*(pw - cam_w)')'; % in camera coords"
"    if pcv(3) <= 0"
"        continue;  % behind camera"
"    end"
"    u = fx*(pcv(1)/pcv(3)) + cx;"
"    v = fy*(pcv(2)/pcv(3)) + cy;"
"    uvAll(i,:) = [u v];"
"    % metric→pixel radius"
"    r_pix = abs(fx*targetRadius/pcv(3));"
"    rr = max(2,round(r_pix));"
"    uu1 = max(1,round(u-rr)); uu2 = min(imgW,round(u+rr));"
"    vv1 = max(1,round(v-rr)); vv2 = min(imgH,round(v+rr));"
"    for yy = vv1:vv2"
"        for xx = uu1:uu2"
"            if (xx - u)^2 + (yy - v)^2 <= rr*rr"
"                frame(yy,xx,1) = 230;   % R"
"                frame(yy,xx,2) = 30;    % G (a bit to look nicer)"
"                frame(yy,xx,3) = 30;    % B"
"            end"
"        end"
"    end"
"end"
""
"end"
];
end

function code = pixelErrorFcn
% MATLAB Function block: choose uv by selectedTargetIdx, compute pixel error
code = [
"function [uErrPix, vErrPix] = fcn(uvAll, selectedTargetIdx, cx, cy)"
"%#codegen"
"uErrPix = 0; vErrPix = 0;"
"idx = int32(selectedTargetIdx);"
"if isempty(uvAll) || any(isnan(uvAll(:)))"
"    return;"
"end"
"if idx < 1 || idx > size(uvAll,1)"
"    idx = 1;"
"end"
"u = uvAll(idx,1); v = uvAll(idx,2);"
"if ~isnan(u) && ~isnan(v)"
"    uErrPix = u - double(cx);"
"    vErrPix = v - double(cy);"
"end"
"end"
];
end

end  % build_uav_virtcam_sim (main)

%% ----------------------- Preview helpers (MATLAB, not Simulink) -----------------------
function [frame, uvAll] = renderVirtualCameraFrame(posENU, targetsENU, fx,fy,cx,cy, imgW,imgH, R_world_from_cv, targetRadius)
R_cv_from_world = R_world_from_cv';
cam_w = posENU(:)';
N = size(targetsENU,1);
frame = uint8(ones(imgH,imgW,3)*40);   % dark gray background
uvAll = nan(N,2);

% center cross
frame(round(cy), max(1,round(cx-6)):min(imgW,round(cx+6)), 2) = 255;
frame(max(1,round(cy-6)):min(imgH,round(cy+6)), round(cx), 2) = 255;

for i=1:N
    pw = targetsENU(i,1:3);
    pcv = (R_cv_from_world*(pw - cam_w)')';
    if pcv(3) <= 0, continue; end
    u = fx*(pcv(1)/pcv(3)) + cx;
    v = fy*(pcv(2)/pcv(3)) + cy;
    uvAll(i,:) = [u v];
    r_pix = abs(fx*targetRadius/pcv(3));
    rr = max(2,round(r_pix));
    uu1 = max(1,round(u-rr)); uu2 = min(imgW,round(u+rr));
    vv1 = max(1,round(v-rr)); vv2 = min(imgH,round(v+rr));
    for yy = vv1:vv2
        for xx = uu1:uu2
            if (xx - u)^2 + (yy - v)^2 <= rr*rr
                frame(yy,xx,1) = 230; frame(yy,xx,2) = 30; frame(yy,xx,3) = 30;
            end
        end
    end
end
end

function bboxes = detectRedBlobs(frame)
% Simple detector that works on our synthetic camera frames
R = frame(:,:,1); G = frame(:,:,2); B = frame(:,:,3);
mask = R > 150 & G < 80 & B < 80;                % threshold "red"
mask = imfill(mask, 'holes');
mask = bwareaopen(mask, 50);
stats = regionprops(mask, 'BoundingBox');
bboxes = vertcat(stats.BoundingBox);
if isempty(bboxes), bboxes = zeros(0,4); end
end

function selectedTargetIdx = uiSelectTarget(frame, bboxes, uvAll)
% Show preview, draw boxes, let user click one; map click → nearest bbox → nearest target center
selectedTargetIdx = [];
fig = figure('Name','Select target to track (click a box)', 'NumberTitle','off');
imshow(frame); hold on;
for k = 1:size(bboxes,1)
    rectangle('Position', bboxes(k,:), 'EdgeColor','y', 'LineWidth', 2);
end
title('Click a yellow box to select target. Press Esc to cancel.');

set(fig,'WindowButtonDownFcn', @mouseClick);
uiwait(fig);

    function mouseClick(~,~)
        cp = get(gca,'CurrentPoint');   % [x y]
        click_xy = cp(1,1:2);
        if isempty(bboxes), uiresume(fig); delete(fig); return; end
        % pick nearest bbox center
        centers = [bboxes(:,1)+bboxes(:,3)/2, bboxes(:,2)+bboxes(:,4)/2];
        [~,iBox] = min(sum((centers - click_xy).^2,2));
        % map bbox center to nearest ground-truth uv to get target index
        [~,iUV] = min(sum((uvAll - centers(iBox,:)).^2,2,'omitnan'));
        if isempty(iUV) || isnan(uvAll(iUV,1)), return; end
        selectedTargetIdx = iUV;
        uiresume(fig); delete(fig);
    end
end
