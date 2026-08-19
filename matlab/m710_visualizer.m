function m710_visualizer(urdfPath)
% M710_VISUALIZER Interactive M-710iC50 URDF visualizer.
% Input angles are FANUC/controller display angles in degrees.
% The project-specific FANUC linkage q3_effective = q3 + q2 is applied.

if nargin < 1 || isempty(urdfPath)
    urdfPath = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), ...
        'robot_test_data', 'm710', 'M-710iC50 M-710iC70', 'urdf', ...
        'M-710iC50 M-710iC70.STEP.SLDASM.urdf');
end
if ~isfile(urdfPath)
    error('找不到 URDF 文件：%s', urdfPath);
end
if exist('importrobot', 'file') ~= 2 || exist('rigidBodyTree', 'class') ~= 8
    error(['需要 Robotics System Toolbox。请在 MATLAB 中安装/启用该工具箱，' ...
        '然后重新运行 m710_visualizer。']);
end

% Resolve package:// mesh URIs to local files so MATLAB can render the meshes.
importPath = urdfPath;
try
    xml = fileread(urdfPath);
    meshRoot = fullfile(fileparts(urdfPath), '..', 'meshes');
    meshRoot = char(java.io.File(meshRoot).getCanonicalPath());
    xml = strrep(xml, 'package://M-710iC50 M-710iC70.obj.SLDASM/meshes/', ...
        [strrep(meshRoot, '\\', '/') '/']);
    importPath = [tempname '.urdf'];
    fid = fopen(importPath, 'w'); fwrite(fid, xml, 'char'); fclose(fid);
catch
    importPath = urdfPath;
end
robot = importrobot(importPath);
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];
target = 'Link_6';
if ~any(strcmp(robot.BodyNames, target))
    error('URDF 中找不到目标 Link_6。可用末端名称：%s', strjoin(robot.BodyNames, ', '));
end

% FANUC controller display limits from the supplied project UI.
limits = [-179.99 179.99; -58.50 74.48; -121.00 228.50; ...
          -358.50 358.50; -123.50 123.50; -358.50 358.50];
qDisplay = [0 -30 45 0 30 0];

fig = uifigure('Name', 'FANUC M-710iC50 正运动学可视化', ...
    'Position', [40 20 1180 820]);
gl = uigridlayout(fig, [1 2]);
gl.ColumnWidth = {'1x', 330};
ax = uiaxes(gl); ax.Layout.Row = 1; ax.Layout.Column = 1;
view(ax, 135, 25); grid(ax, 'on'); axis(ax, 'equal');
xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
meshFig = figure('Name', 'M-710iC50 实体网格', 'Color', 'w', ...
    'Position', [80 80 800 700]);
meshAx = axes('Parent', meshFig);

panel = uipanel(gl, 'Title', 'FANUC 关节角（度）');
panel.Layout.Row = 1; panel.Layout.Column = 2;
pg = uigridlayout(panel, [18 3]);
rows = repmat({28}, 1, 18);
rows{7} = 10; rows{9} = 60; rows{11} = 150; rows{13} = 55;
rows{15} = 34; rows{16} = 34; rows{18} = 60;
pg.RowHeight = rows;
pg.ColumnWidth = {45, '1x', 85};
sliders = gobjects(1, 6); values = gobjects(1, 6);
for i = 1:6
    jl = uilabel(pg, 'Text', sprintf('J%d', i), 'HorizontalAlignment', 'center');
    jl.Layout.Row = i; jl.Layout.Column = 1;
    sliders(i) = uislider(pg, 'Limits', limits(i,:), 'Value', qDisplay(i), ...
        'MajorTicks', [], 'ValueChangedFcn', @update); 
    sliders(i).Layout.Row = i; sliders(i).Layout.Column = 2;
    values(i) = uieditfield(pg, 'numeric', 'Value', qDisplay(i), ...
        'Limits', limits(i,:), 'ValueDisplayFormat', '%.3f', 'ValueChangedFcn', @editAngle);
    values(i).Layout.Row = i; values(i).Layout.Column = 3;
end

heading1 = uilabel(pg, 'Text', '有效 URDF 关节角', 'FontWeight', 'bold');
heading1.Layout.Row = 8; heading1.Layout.Column = [1 3];
effectiveText = uilabel(pg, 'Text', '', 'WordWrap', 'on');
effectiveText.Layout.Row = 9; effectiveText.Layout.Column = [1 3];
heading2 = uilabel(pg, 'Text', 'Link_6 末端位姿', 'FontWeight', 'bold');
heading2.Layout.Row = 10; heading2.Layout.Column = [1 3];
poseText = uilabel(pg, 'Text', '', 'WordWrap', 'on');
poseText.Layout.Row = 11; poseText.Layout.Column = [1 3];
note = uilabel(pg, 'Text', '说明：FANUC 联动规则为 q3_URDF = J3 + J2。', ...
    'FontColor', [0.25 0.25 0.25], 'WordWrap', 'on');
note.Layout.Row = 13; note.Layout.Column = [1 3];
resetButton = uibutton(pg, 'Text', '重置', 'ButtonPushedFcn', @resetAngles);
resetButton.Layout.Row = 15; resetButton.Layout.Column = [1 3];
copyButton = uibutton(pg, 'Text', '复制末端位姿', 'ButtonPushedFcn', @copyPose);
copyButton.Layout.Row = 16; copyButton.Layout.Column = [1 3];
statusText = uilabel(pg, 'Text', ['URDF: ' urdfPath], 'WordWrap', 'on', 'FontSize', 10);
statusText.Layout.Row = 18; statusText.Layout.Column = [1 3];

update([], []);

    function editAngle(src, ~)
        idx = find(values == src, 1);
        sliders(idx).Value = min(max(src.Value, limits(idx,1)), limits(idx,2));
        qDisplay(idx) = sliders(idx).Value;
        update();
    end

    function update(~, ~)
        for k = 1:6
            qDisplay(k) = sliders(k).Value;
            values(k).Value = qDisplay(k);
        end
        qEffective = qDisplay;
        qEffective(3) = qDisplay(3) + qDisplay(2);
        qRadians = deg2rad(qEffective);
        cla(ax);
        figure(meshFig);
        meshAx = show(robot, qRadians, 'PreservePlot', false, ...
            'Frames', 'off', 'Visuals', 'on');
        view(meshAx, 135, 25); grid(meshAx, 'on'); axis(meshAx, 'equal');
        xlim(meshAx, [-2.5 2.5]); ylim(meshAx, [-2.5 2.5]); zlim(meshAx, [-0.5 3.5]);
        xlabel(meshAx, 'X (m)'); ylabel(meshAx, 'Y (m)'); zlabel(meshAx, 'Z (m)');
        T = getTransform(robot, qRadians, target);
        p = T(1:3,4) * 1000;
        effectiveText.Text = sprintf('J1 %.3f   J2 %.3f   J3 %.3f\nJ4 %.3f   J5 %.3f   J6 %.3f', qEffective);
        wpr = rotationToFanucWPR(T(1:3,1:3));
        poseText.Text = sprintf('X %.3f mm\nY %.3f mm\nZ %.3f mm\nW %.3f°   P %.3f°   R %.3f°', ...
            p(1), p(2), p(3), wpr(1), wpr(2), wpr(3));
        drawnow limitrate;
    end

    function resetAngles(~, ~)
        qDisplay = [0 -30 45 0 30 0];
        for k = 1:6, sliders(k).Value = qDisplay(k); end
        update([], []);
    end

    function copyPose(~, ~)
        txt = sprintf('%s\n%s', effectiveText.Text, poseText.Text);
        clipboard('copy', txt);
        statusText.Text = '末端位姿已复制到剪贴板。';
    end
end

function wpr = rotationToFanucWPR(R)
% FANUC WPR: R = Rz(R) * Ry(P) * Rx(W).
P = asin(max(-1, min(1, -R(3,1))));
if abs(cos(P)) > 1e-10
    W = atan2(R(3,2), R(3,3));
    Yaw = atan2(R(2,1), R(1,1));
else
    W = atan2(-R(2,3), R(2,2));
    Yaw = 0;
end
wpr = rad2deg([W P Yaw]);
end
