% uav_attached_camera_pid.m
% Virtual 3D UAV with camera physically attached to the bottom, downward-looking camera,
% color-based detection, manual selection, and full PID in world frame (X,Y,Z).
%
% Requirements: MATLAB. Computer Vision Toolbox is NOT required but may help with image display tools.
% Run: save and run in MATLAB. Click detections in the camera pane to select. Press 'q' to quit.

function uav_attached_camera_pid()
clc; close all;

%% ---------------- Simulation parameters ----------------
dt       = 0.03;         % timestep [s]
simTime  = 40;           % total sim time [s]
steps    = round(simTime/dt);

% Drone initial state (world inertial ENU): X east, Y north, Z up (meters)
dronePos = [0; 0; 6];      % start 6m above ground
droneYaw = 0;              % radians (not used for rotation here, but included for completeness)

% Targets (world coordinates) [X Y Z]
targets = [5 5 0;
           2 7 0;
           7 2 0];
nTargets = size(targets,1);
targetRadius_m = 0.30;     % physical radius (m) - used for depth estimation

% Drone geometry
drone_dx = 0.6; drone_dy = 0.6; drone_dz = 0.25;

% Camera physically attached to bottom of drone:
cam_offset_body = [0; 0; -0.15];   % camera position relative to drone body (m) (bottom)
% camera points along body -Z (downwards). We'll treat camera orientation aligned with world axes except pointing down.

% Camera intrinsics (synthetic pinhole)
imgW = 960; imgH = 720;
fovV_deg = 60;
focalPx = (imgH/2)/tand(fovV_deg/2);
cx = imgW/2; cy = imgH/2;

% PID gains for X, Y, Z (tune as needed)
Kp_xy = 0.6; Ki_xy = 0.02; Kd_xy = 0.08;
Kp_z  = 0.8; Ki_z  = 0.03; Kd_z  = 0.12;

vmax = 3.0;   % m/s max horizontal velocity
vzmax = 1.0;  % m/s max vertical velocity

% Target color rendering (so detection is easy)
targetColors = lines(nTargets);

%% ---------------- Setup figure and 3D scene ----------------
set(0,'DefaultFigureRenderer','opengl');
fig = figure('Name','UAV (camera attached below) + PID','NumberTitle','off',...
    'Position',[40 40 1700 900],'Color',[0.08 0.08 0.09]);

% Left: 3D axes: world
ax3 = axes('Parent',fig,'Position',[0.03 0.06 0.52 0.88]);
hold(ax3,'on'); grid(ax3,'on'); axis(ax3,'equal');
xlabel(ax3,'X (m)'); ylabel(ax3,'Y (m)'); zlabel(ax3,'Z (m)');
xlim(ax3,[-2 10]); ylim(ax3,[-2 10]); zlim(ax3,[0 10]);
title(ax3,'World (ENU) - Left: 3D Scene','Color','w');
ax3.Color = [0.04 0.04 0.06];

% Ground plane
[Xg,Yg] = meshgrid(-12:1:12, -12:1:12);
surf(ax3,Xg,Yg,zeros(size(Xg)),'FaceColor',[0.12 0.16 0.12],'EdgeColor','none');

% Drone (box) draw
hDrone = drawBox(ax3, dronePos, drone_dx, drone_dy, drone_dz, [0 0.5 1]);

% Targets: cylinders + colored disk (billboard)
hTargets = gobjects(nTargets,1);
hDisks = gobjects(nTargets,1);
for i=1:nTargets
    pos = targets(i,:);
    hTargets(i) = drawCylinder(ax3, pos + [0 0 0.12], targetRadius_m, 0.24, targetColors(i,:));
    hDisks(i) = drawDiskBillboard(ax3, pos, targetRadius_m, targetColors(i,:));
end

% Right: camera feed axes
axCam = axes('Parent',fig,'Position',[0.58 0.06 0.39 0.88]);
hCam = imshow(zeros(imgH,imgW,3,'uint8'),'Parent',axCam);
title(axCam,'Downward Camera (attached to bottom of drone)','Color','w');
axCam.Visible = 'off';

% instructions
annotation('textbox',[0.58 0.92 0.39 0.06],'String','Click a yellow detection to select object. Press Q to quit.','EdgeColor','none','Color','w','FontSize',11,'HorizontalAlignment','center');

% Storage for detections and selection
lastBBoxes = zeros(0,4);
lastCentroids = zeros(0,2);
selectedIdx = []; % index in lastBBoxes
selectedTargetWorldIdx = []; % index into targets world array

% PID integrators
int_xy = [0;0]; prev_xy = [0;0];
int_z  = 0; prev_z = 0;

% set click callback for selection
set(fig,'WindowButtonDownFcn',@(s,e) pickFromCam());

drawnow;

%% ---------------- Simulation loop ----------------
for step = 1:steps
    % ------- 1) compute camera pose from drone pose (camera rigidly attached under drone) -------
    % dronePos in world; camera position = dronePos + cam_offset_body rotated by droneYaw (if yaw used)
    % For simplicity, we assume no yaw rotation. If you add yaw, rotate offset.
    camPos = dronePos + cam_offset_body;   % world coordinates of camera origin
    % camera optical axis points down (world -Z)
    camTarget = camPos + [0;0;-1];
    camUp = [0 1 0];
    
    % Configure 3D camera viewpoint to match camera pose:
    camva(ax3, fovV_deg);
    campos(ax3, camPos'); camtarget(ax3, camTarget'); camup(ax3, camUp');
    
    % Render and capture axes image (best approach: getframe of axes)
    drawnow limitrate;
    try
        frm = getframe(ax3);            % capture exactly axes view
        camFull = frm.cdata;
    catch
        % fallback to whole figure crop
        frm = getframe(fig); raw = frm.cdata;
        axpos = getpixelposition(ax3,true);
        figpx = getpixelposition(fig,true);
        cropRect = [round(axpos(1)), round(figpx(4) - (axpos(2)+axpos(4))), round(axpos(3)), round(axpos(4))];
        try
            camFull = imcrop(raw, cropRect);
        catch
            camFull = raw;
        end
    end
    camFrame = imresize(camFull, [imgH imgW]);
    
    % ------- 2) color-based detection in camFrame -------
    rgb = im2double(camFrame);
    maxc = max(rgb,[],3);
    minc = min(rgb,[],3);
    sat = maxc - minc;
    % choose bright saturated regions (these are our colored targets)
    mask = (maxc > 0.25) & (sat > 0.06);
    mask = imfill(mask,'holes');
    mask = bwareaopen(mask, 100);
    stats = regionprops(mask, 'BoundingBox','Centroid','Area');
    if isempty(stats)
        lastBBoxes = zeros(0,4);
        lastCentroids = zeros(0,2);
    else
        lastBBoxes = vertcat(stats.BoundingBox);
        lastCentroids = vertcat(stats.Centroid);
    end
    
    % overlay detection boxes
    dispFrame = camFrame;
    for b = 1:size(lastBBoxes,1)
        dispFrame = insertShape(dispFrame,'Rectangle',lastBBoxes(b,:),'Color','yellow','LineWidth',3);
    end
    
    % If no selection yet, auto choose largest region
    if isempty(selectedIdx) && ~isempty(stats)
        areas = [stats.Area]';
        [~,imax] = max(areas);
        selectedIdx = imax;
        % map to nearest world target by projection
        uvTargets = worldToImageAll(targets, dronePos, focalPx, imgW, imgH); % returns Nx2
        if ~isempty(lastCentroids)
            d2 = sum((uvTargets - lastCentroids(selectedIdx,:)).^2,2,'omitnan');
            [~,mmin] = min(d2); selectedTargetWorldIdx = mmin;
        else
            selectedTargetWorldIdx = 1;
        end
    end
    
    % highlight selection (green)
    if ~isempty(selectedIdx) && selectedIdx <= size(lastBBoxes,1)
        dispFrame = insertShape(dispFrame,'Rectangle',lastBBoxes(selectedIdx,:),'Color','green','LineWidth',4);
    end
    
    % display camera feed
    set(hCam,'CData',dispFrame);
    
    % ------- 3) If selected compute pixel->camera->world conversion -------
    if ~isempty(selectedIdx) && selectedIdx <= size(lastBBoxes,1)
        bb = lastBBoxes(selectedIdx,:);
        u = bb(1) + bb(3)/2; v = bb(2) + bb(4)/2;
        r_pix = max(bb(3), bb(4))/2;
        % depth estimate using known radius: Z = (real_radius * focal) / r_pix
        Z_est = (targetRadius_m * focalPx) / max(r_pix,1e-3);  % meters in camera forward direction
        % pixel -> camera coords: x_cam (right), y_cam (down), z_cam forward
        x_cam = (u - cx) * (Z_est / focalPx);
        y_cam = (v - cy) * (Z_est / focalPx);
        z_cam = Z_est;
        % camera frame to world: camera points down => camera forward = -world Z
        % camera axes relative to world: x_cam -> +X_world, y_cam -> +Y_world, z_cam -> -Z_world
        R_w_from_cam = [1 0 0; 0 1 0; 0 0 -1];
        obj_world = R_w_from_cam * [x_cam; y_cam; z_cam] + camPos;
        err_world_xy = obj_world(1:2) - dronePos(1:2); % target relative to drone in world XY
        err_z = obj_world(3) - dronePos(3);
    else
        err_world_xy = [0;0];
        err_z = 0;
        obj_world = [NaN;NaN;NaN];
        Z_est = NaN;
    end
    
    % ------- 4) PID controllers produce velocity commands -------
    % XY PID
    if ~isempty(selectedIdx)
        err = err_world_xy;
        int_xy = int_xy + err * dt;
        deriv_xy = (err - prev_xy) / dt;
        u_xy = Kp_xy * err + Ki_xy * int_xy + Kd_xy * deriv_xy;  % desired velocity (m/s)
        prev_xy = err;
    else
        u_xy = [0;0];
    end
    vx = max(-vmax, min(vmax, -u_xy(1)));  % negative sign because pixel->error directions earlier
    vy = max(-vmax, min(vmax, -u_xy(2)));
    
    % Z PID (altitude control)
    desiredAlt = targetRadius_m + 0.5; % example desired altitude above ground or object (tweakable)
    if ~isempty(selectedIdx)
        ez = (obj_world(3) + desiredAlt) - dronePos(3);  % we want to be desiredAlt above object
        int_z = int_z + ez * dt;
        deriv_z = (ez - prev_z) / dt;
        uz = Kp_z * ez + Ki_z * int_z + Kd_z * deriv_z;  % m/s vertical
        prev_z = ez;
    else
        uz = 0;
    end
    vz = max(-vzmax, min(vzmax, uz));
    
    % ------- 5) integrate drone velocities to update state -------
    dronePos = dronePos + [vx; vy; vz] * dt;
    % clamp world boundaries (optional)
    dronePos(1) = max(-5, min(12, dronePos(1)));
    dronePos(2) = max(-5, min(12, dronePos(2)));
    dronePos(3) = max(0.5, min(20, dronePos(3)));
    
    % update 3D drone drawing
    updateBox(hDrone, dronePos, drone_dx, drone_dy, drone_dz);
    
    % ------- 6) overlay some camera visuals (line, triangle) -------
    if ~isempty(selectedIdx) && ~isempty(lastCentroids)
        centers = lastCentroids(selectedIdx,:);
        u0 = cx; v0 = cy;
        u = centers(1); v = centers(2);
        % draw center-to-object line (yellow)
        numPts = max(3,round(hypot(u-u0, v-v0)));
        for kk=0:numPts
            xk = round(u0 + (u - u0)*(kk/numPts));
            yk = round(v0 + (v - v0)*(kk/numPts));
            if xk>=1 && xk<=imgW && yk>=1 && yk<=imgH
                dispFrame(yk,xk,:) = uint8([255 255 0]);
            end
        end
        % horizontal & vertical legs
        for xk = round(min(u0,u)):round(max(u0,u))
            yk = round(v0);
            if xk>=1 && xk<=imgW && yk>=1 && yk<=imgH
                dispFrame(yk,xk,:) = uint8([0 120 255]);
            end
        end
        for yk = round(min(v0,v)):round(max(v0,v))
            xk = round(u);
            if xk>=1 && xk<=imgW && yk>=1 && yk<=imgH
                dispFrame(yk,xk,:) = uint8([255 60 60]);
            end
        end
        set(hCam,'CData',dispFrame);
    end
    
    % ------- 7) Terminal output (units, clear) -------
    if ~isempty(selectedIdx)
        pitch_deg = atan2d(err_world_xy(2), dronePos(3));  % small-angle proxy
        roll_deg  = atan2d(err_world_xy(1), dronePos(3));
        distXY = hypot(err_world_xy(1), err_world_xy(2));
        fprintf('Step %04d | Pitch: %6.2f° | Roll: %6.2f° | Alt: %5.2f m | DistXY: %6.3f m | vx: %5.3f vy: %5.3f (m/s) | vz: %5.3f m/s | Zest: %5.3f m\n', ...
            step, pitch_deg, roll_deg, dronePos(3), distXY, vx, vy, vz, Z_est);
    else
        fprintf('Step %04d | No selection | DronePos (m): [%.2f, %.2f, %.2f]\n', step, dronePos(1), dronePos(2), dronePos(3));
    end
    
    drawnow limitrate;
    if ~ishandle(fig), break; end
    ch = get(fig,'CurrentCharacter');
    if ~isempty(ch) && (ch == 'q' || ch == 'Q'), break; end
end

disp('Simulation finished.');
if ishandle(fig), close(fig); end
end % main

%% ---------------- Helper functions ----------------

function h = drawBox(ax,pos,dx,dy,dz,color)
[X,Y,Z] = ndgrid([-dx dx]/2,[-dy dy]/2,[-dz dz]/2);
verts = [X(:) Y(:) Z(:)];
faces = convhull(verts);
h = patch(ax,'Vertices',verts + pos','Faces',faces,'FaceColor',color,'EdgeColor','none','FaceAlpha',0.95);
end

function updateBox(h,pos,dx,dy,dz)
[X,Y,Z] = ndgrid([-dx dx]/2,[-dy dy]/2,[-dz dz]/2);
verts = [X(:) Y(:) Z(:)];
if isprop(h,'Vertices'), h.Vertices = verts + pos'; end
end

function h = drawCylinder(ax, center, r, hgt, color)
[th,z] = meshgrid(linspace(0,2*pi,36), linspace(0,hgt,2));
X = center(1) + r*cos(th);
Y = center(2) + r*sin(th);
Z = center(3) + z - hgt/2;
h = surf(ax,X,Y,Z, 'FaceColor', color,'EdgeColor','none','FaceAlpha',1);
end

function h = drawDiskBillboard(ax, center, radius, color)
theta = linspace(0,2*pi,60);
x = center(1) + radius*cos(theta);
y = center(2) + radius*sin(theta);
z = zeros(size(x)) + 0.01;
h = patch(ax,x,y,z,color,'EdgeColor','none','FaceAlpha',1);
end

function uv = worldToImageAll(worldPts, dronePos, focalPx, imgW, imgH)
% project world points into image for a camera pointing straight down at dronePos
R_cv_from_world = [1 0 0; 0 1 0; 0 0 -1];
N = size(worldPts,1);
uv = nan(N,2);
for i=1:N
    pw = worldPts(i,:);
    pcv = (R_cv_from_world*(pw - dronePos)')';
    if pcv(3) <= 0
        uv(i,:) = [NaN NaN];
        continue;
    end
    u = focalPx * (pcv(1)/pcv(3)) + imgW/2;
    v = focalPx * (pcv(2)/pcv(3)) + imgH/2;
    uv(i,:) = [u v];
end
end

function pickFromCam()
% Callback: choose the detection on click
fig = gcf;
axCam = findobj(fig,'Type','axes','Tag',''); % not robust but our right axes is only axes with imshow
% get click in axes coordinates
cp = get(gca,'CurrentPoint'); % might be axes at click; ensure image axes
xClick = cp(1,1); yClick = cp(1,2);
% Find lastBBoxes and lastCentroids in caller workspace
s = dbstack('-completenames'); %#ok<NASGU>
% We will use findall for imshow handle and get CData to figure mapping
% Simpler: use global variables by storing selection inside base workspace (quick hack)
% But to keep this self-contained, we map selections by checking bounding boxes in display.
% For simplicity, this callback will set the selectedIdx in the main function by updating figure's UserData.
ud = get(fig,'UserData');
% If lastBBoxes exists
if isfield(ud,'lastBBoxes') && ~isempty(ud.lastBBoxes)
    bboxes = ud.lastBBoxes;
    centers = ud.lastCentroids;
    idx = [];
    for i=1:size(bboxes,1)
        bb = bboxes(i,:);
        if xClick >= bb(1) && xClick <= bb(1)+bb(3) && yClick >= bb(2) && yClick <= bb(2)+bb(4)
            idx = i; break;
        end
    end
    if isempty(idx)
        d = hypot(centers(:,1)-xClick, centers(:,2)-yClick);
        [~,idx] = min(d);
    end
    ud.selectedIdx = idx;
    set(fig,'UserData',ud);
    fprintf('User clicked selection: %d\n', idx);
end
end
