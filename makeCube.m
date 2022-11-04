%Plots the cube of the drone and the axis of the orientation

function makeCube(results)

%Extract results
t = results.t;
states = results.states;
numpoints = length(t);


figure(1)
hold on

for i = 1:numpoints
    clf

    xC = states(8,i);
    yC = states(9,i);
    zC = states(10,i);
    euls = quat2eul(states(1:4,i)');
    euls = rad2deg(euls);

    plot3(0,0,0)

    drawCube([xC, yC, zC, 1, euls(3), euls(2), euls(1)])

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

    F(i) = getframe;
end

video = VideoWriter('test.avi', 'Uncompressed AVI');
video.FrameRate = 60;
open(video)
writeVideo(video, frame);
close(video)