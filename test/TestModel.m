clear all
P.Surf(1).b=1;          % Span
P.Surf(1).c=0.4;        % Chord
P.Surf(1).Pos=[0;0;0];  % Position
P.Surf(1).Rot=[0;0;0];  % Rotation
P.Surf(1).CLa=5;        % C_L_alpha
P.Surf(1).e=0.9;        % Efficiency factor
P.Surf(1).AR=P.Surf(1).b*2/P.Surf(1).c; %Aspect ratio
P.Surf(1).Cma=1.5;        % C_m_alpha
P.Surf(1).M=50;         % Blending factor
P.Surf(1).a0=0.3;       % Stall point

P.Surf(2).b=1;          % Span
P.Surf(2).c=0.4;        % Chord
P.Surf(2).Pos=[0;0;0];  % Position
P.Surf(2).Rot=[0;0;0];  % Rotation
P.Surf(2).CLa=5;        % C_L_alpha
P.Surf(2).e=0.9;        % Efficiency factor
P.Surf(2).AR=P.Surf(1).b*2/P.Surf(1).c; %Aspect ratio
P.Surf(2).Cma=1.5;        % C_m_alpha
P.Surf(2).M=50;         % Blending factor
P.Surf(2).a0=0.3;       % Stall point

% Inertial parameter
P.mass=13.5;
P.Ix=0.8;
P.Iy=1.1;
P.Iz=1.7;
P.Ixz=0.12;
P.CoG=[0;0;0];

% Constants
P.g=9.80665;
P.rho=1.225;

% Move origin to CoG
n=size(P.Surf,2);
for i=1:n
    P.Surf(i).Pos=P.Surf(i).Pos-P.CoG;
end

%Initial state
P.x0= [
    0;0;0;  %ned
    20;0;0;  %uvw
    0;1/180*pi;0;  %euler
    0;0;0]; %pqr
