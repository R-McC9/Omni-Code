%% PROGRAM INFORMATION
% FILENAME: 
% DESCRIPTION: 
% INPUTS: 
% OUTPUTS: 
% AUTHOR: Michal Rittikaidachar
% REVISIION HISTORY:`REV A - x/x/x
% NOTES: 
function droneAnimation(results, params, saveVideo)



% Unpack Results 
t = results.t;
states = results.states;
ref = results.ref;
numPoints = length(t);

% Unpack State Matrix
xB= states(8,:);
yB= states(9,:);
zB= states(10,:);
eulerParams = states(  1:4,:)';

%Unpack reference trajectory
xRef = ref(5,:);
yRef = ref(6,:);
zRef = ref(7,:);
quatRef = ref(1:4,:)';

% Convert Euler Parameters (quaternion) to Body Fixed ZYX Euler Angles
eulerAngles = quat2eul(eulerParams);
qBz= eulerAngles(:,1);
qBy= eulerAngles(:,2);
qBx= eulerAngles(:,3);

% Convert reference trajectory angles (quaternion) to euler angels
eulerRef = quat2eul(quatRef);
qRefz = eulerRef(:,1);
qRefy = eulerRef(:,2);
qRefx = eulerRef(:,3);

% Unpack Desired State/Trajectory
%stateDes = results.stateDes;

% Unpack Animation Parameters
 animationTimeScale = params.animationTimeScale;
 stepSize = (t(end)- t(1)) / (length(t)-1);
 frameRate = round(1/stepSize) * animationTimeScale;
vehicleDim = [1, 1, 1];
targetDim = [0.75, 0.75, 0.75];

if saveVideo
  videoName = params.videoName;
  myVideo = VideoWriter(videoName);
  myVideo.FrameRate = frameRate;
  open(myVideo)
end

%% Generate Initial Plots 
animationFigure = figure('units','normalized','outerposition',[0 0 1 1]);
figure(animationFigure);
%setPointPlot = plot3(setPoint(1),setPoint(2),setPoint(3), 'r.','markersize',25);
view(45,45)
hold on
grid on

ax = gca;
ax.GridAlpha = 0.3;

%Actual UAV
[bodyVertices, bodyFaces] =  plotCube(vehicleDim,[xB(1), yB(1), zB(1)], [qBz(1), qBy(1), qBx(1)]);
bodyPlot = patch('Vertices',bodyVertices,'Faces',bodyFaces, 'facealpha', 0.25);

%Target Position
[targVertices, targFaces] = plotCube(targetDim, [xRef(1), yRef(1), zRef(1)], [qRefz(1), qRefy(1), qRefx(1)]);
targPlot = patch('Vertices',targVertices,'Faces',targFaces,'facealpha',0.25,'FaceColor','red');

%Path
pathPlot = plot3(xB(1), yB(1), zB(1), 'k--', 'linewidth',1);

%Target Path
targPathPlot = plot3(xRef(1), yRef(1), zRef(1), 'r--', 'linewidth',1)

xMin = -12;
xMax = 12;
yMin = -12;
yMax = 12;
zMin = -12;
zMax = 12;
axis([xMin xMax yMin yMax zMin zMax]);
xlabel('x Position [m]');
ylabel('y Position [m]');
zlabel('z Position [m]');
%legend('Set Point', 'VehicleBody','Drone X', 'Drone Y', 'Drone Z')

%% Step through plot for animation
for i = 1:numPoints
%     setPointPlot.XData = setPoint(i,1);
%     setPointPlot.YData = setPoint(i,2);
%     setPointPlot.ZData = setPoint(i,3);


    %Plot Actual UAV
    bodyPlot.XData = xB(i);
    bodyPlot.YData = yB(i);
    bodyPlot.ZData = zB(i);
    [ bodyPlot.Vertices, bodyPlot.Faces] =  plotCube(vehicleDim,[xB(i), yB(i), zB(i)], [qBz(i), qBy(i), qBx(i)]);

    %Plot Target UAV
    targPlot.XData = xRef(i);
    targPlot.YData = yRef(i);
    targPlot.ZData = zRef(i);
    [targPlot.Vertices, targPlot.Faces] = plotCube(targetDim,[xRef(i), yRef(i), zRef(i)], [qRefz(i), qRefy(i), qRefx(i)]);

    %Plot Actual UAV reference frame with geom3D
    axisX = drawVector3d([xB(i), yB(i), zB(i)], [1.2 0 0]*eul2rotm([qBz(i), qBy(i), qBx(i)]), 'color', 'r', 'linewidth', 2);
    axisY = drawVector3d([xB(i), yB(i), zB(i)], [0 1.2 0]*eul2rotm([qBz(i), qBy(i), qBx(i)]), 'color', 'g', 'linewidth', 2);
    axisZ = drawVector3d([xB(i), yB(i), zB(i)], [0 0 1.2]*eul2rotm([qBz(i), qBy(i), qBx(i)]), 'color', 'b', 'linewidth', 2);

    %Plot drone xy location
    xyPos = drawEdge3d([xB(i), yB(i), zB(i), xB(i), yB(i), -12], 'color', 'c');
    xyPoint = drawPoint3d(xB(i), yB(i), -12, 'color', 'm');
   
    %Plot actual path
    pathPlot.XData = xB(1:i);
    pathPlot.YData = yB(1:i);
    pathPlot.ZData = zB(1:i);

    %Plot Reference path
    targPathPlot.XData = xRef(1:i);
    targPathPlot.YData = yRef(1:i);
    targPathPlot.ZData = zRef(1:i);

    legend('Drone Body', 'Target Body','Drone Path','Target Path','Drone X', 'Drone Y', 'Drone Z')
    
    drawnow

    if saveVideo
      frame = getframe(animationFigure);
      writeVideo(myVideo,frame);
    end

    delete(axisX)
    delete(axisY)
    delete(axisZ)
    delete(xyPos)
    delete(xyPoint)
    
end
%% Save Video and Cleanup 
if saveVideo
  close(myVideo)
end


end