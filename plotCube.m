function [vertices, faces]= plotCube(sideLengths, position, orientation)
lx = sideLengths(1);
ly = sideLengths(2);
lz = sideLengths(3);
origin = position;
qZ = orientation(1);
qy = orientation(2);
qX = orientation(3);

p1 = [-lx/2 -ly/2 -lz/2 ];
p2 = [lx/2, -ly/2, -lz/2 ];
p3 = [lx/2, ly/2, -lz/2 ];
p4 = [-lx/2, ly/2, -lz/2 ];
p5 = [-lx/2, -ly/2, lz/2 ];
p6 = [lx/2, -ly/2, lz/2 ];
p7 = [lx/2, ly/2, lz/2 ];
p8 = [-lx/2, ly/2, lz/2];

verticesRaw = [ p1; p2; p3; p4; p5; p6; p7; p8];

Rzyx(1,1) = cos(qy)*cos(qZ);
Rzyx(1,2) = sin(qZ)*cos(qy);
Rzyx(1,3) = -sin(qy);
Rzyx(2,1) = sin(qX)*sin(qy)*cos(qZ) - sin(qZ)*cos(qX);
Rzyx(2,2) = cos(qX)*cos(qZ) + sin(qX)*sin(qy)*sin(qZ);
Rzyx(2,3) = sin(qX)*cos(qy);
Rzyx(3,1) = sin(qX)*sin(qZ) + sin(qy)*cos(qX)*cos(qZ);
Rzyx(3,2) = sin(qy)*sin(qZ)*cos(qX) - sin(qX)*cos(qZ);
Rzyx(3,3) = cos(qX)*cos(qy);

vertices = zeros(size(verticesRaw));
for i = 1: size(vertices,1)
  vertices(i,:) =  (Rzyx*verticesRaw(i,:)')' + origin;
end
faces = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8] ;

%cubePlot = patch('Vertices',vertices,'Faces',faces, 'FaceColor', [0.5, 0.5, 0.5]);

end

 
 
