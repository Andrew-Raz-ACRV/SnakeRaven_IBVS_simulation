function plotprism(T,x,y,z,W,L,H,C)
%Plots a prism at point x,y,z in frame T with dimensions W,L,H ~ sides in x,y,z
%with colour C

%Transparency
a = 0.2;

%bottom side
X = [x, x+W, x+W, x];
Y = [y y y+L y+L];
Z = [z z z z];

P = TransformPoints(T,[X',Y',Z']);
X = P(:,1); Y = P(:,2); Z = P(:,3);
p1 = patch(X,Y,Z,C);
p1.FaceVertexAlphaData = a;    % Set constant transparency 
p1.FaceAlpha = 'flat' ;          % Interpolate to find face transparency

%Top side
X = [x, x+W, x+W, x];
Y = [y y y+L y+L];
Z = [z+H z+H z+H z+H];

P = TransformPoints(T,[X',Y',Z']);
X = P(:,1); Y = P(:,2); Z = P(:,3);
p1 = patch(X,Y,Z,C);
p1.FaceVertexAlphaData = a;    % Set constant transparency 
p1.FaceAlpha = 'flat' ;          % Interpolate to find face transparency


%Front side
X = [x, x+W, x+W, x];
Y = [y y y y];
Z = [z z z+H z+H];

P = TransformPoints(T,[X',Y',Z']);
X = P(:,1); Y = P(:,2); Z = P(:,3);
%patch(X,Y,Z,C);
p1 = patch(X,Y,Z,C);
p1.FaceVertexAlphaData = a;    % Set constant transparency 
p1.FaceAlpha = 'flat' ;          % Interpolate to find face transparency

%back side
X = [x, x+W, x+W, x];
Y = [y+L y+L y+L y+L];
Z = [z z z+H z+H];

P = TransformPoints(T,[X',Y',Z']);
X = P(:,1); Y = P(:,2); Z = P(:,3);
p1 = patch(X,Y,Z,C);
p1.FaceVertexAlphaData = a;    % Set constant transparency 
p1.FaceAlpha = 'flat' ;          % Interpolate to find face transparency

%Left side
X = [x x x x];
Y = [y y+L y+L y];
Z = [z z z+H z+H];

P = TransformPoints(T,[X',Y',Z']);
X = P(:,1); Y = P(:,2); Z = P(:,3);
p1 = patch(X,Y,Z,C);
p1.FaceVertexAlphaData = a;    % Set constant transparency 
p1.FaceAlpha = 'flat' ;          % Interpolate to find face transparency

%Right side
X = [x+W x+W x+W x+W];
Y = [y y+L y+L y];
Z = [z z z+H z+H];

P = TransformPoints(T,[X',Y',Z']);
X = P(:,1); Y = P(:,2); Z = P(:,3);
p1 = patch(X,Y,Z,C);
p1.FaceVertexAlphaData = a;    % Set constant transparency 
p1.FaceAlpha = 'flat' ;          % Interpolate to find face transparency




end