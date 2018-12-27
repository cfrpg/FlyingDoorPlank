% Surface
% Right wing
P.Surf(1).b=1;          % Span
P.Surf(1).c=0.4;        % Chord
P.Surf(1).Pos=[0;P.Surf(1).b/2+0.1;0];  % Position
P.Surf(1).Rot=[0;0;0];  % Rotation
P.Surf(1).CLa=5;        % C_L_alpha
P.Surf(1).e=0.9;        % Efficiency factor
P.Surf(1).AR=P.Surf(1).b*2/P.Surf(1).c; %Aspect ratio
P.Surf(1).Cma=1.5;        % C_m_alpha
P.Surf(1).M=50;         % Blending factor
P.Surf(1).a0=0.3;       % Stall point

% Left wing
P.Surf(2).b=1;          % Span
P.Surf(2).c=0.4;        % Chord
P.Surf(2).Pos=[0;-P.Surf(2).b/2-0.1;0];  % Position
P.Surf(2).Rot=[0;0;0];  % Rotation
P.Surf(2).CLa=5;        % C_L_alpha
P.Surf(2).e=0.9;        % Efficiency factor
P.Surf(2).AR=P.Surf(1).b*2/P.Surf(1).c; %Aspect ratio
P.Surf(2).Cma=1.5;        % C_m_alpha
P.Surf(2).M=50;         % Blending factor
P.Surf(2).a0=0.3;       % Stall point


% HTail
P.Surf(3).b=1;          % Span
P.Surf(3).c=0.4;        % Chord
P.Surf(3).Pos=[-1.5;0;0];  % Position
P.Surf(3).Rot=[0;0;0];  % Rotation
P.Surf(3).CLa=5;        % C_L_alpha
P.Surf(3).e=0.9;        % Efficiency factor
P.Surf(3).AR=P.Surf(1).b/P.Surf(1).c; %Aspect ratio
P.Surf(3).Cma=1.5;        % C_m_alpha
P.Surf(3).M=50;         % Blending factor
P.Surf(3).a0=0.3;       % Stall point

% VTail
P.Surf(4).b=0.6;        % Span
P.Surf(4).c=0.4;        % Chord
P.Surf(4).Pos=[-1.5;0;-P.Surf(4).b/2];  % Position
P.Surf(4).Rot=[-pi/2;0;0];  % Rotation
P.Surf(4).CLa=5;        % C_L_alpha
P.Surf(4).e=0.9;        % Efficiency factor
P.Surf(4).AR=P.Surf(1).b*2/P.Surf(1).c; %Aspect ratio
P.Surf(4).Cma=1.5;        % C_m_alpha
P.Surf(4).M=50;         % Blending factor
P.Surf(4).a0=0.3;       % Stall point

% Prop parameter


% Inertial parameter
P.mass=0;
P.Ix=0;
P.Iy=0;
P.Iz=0;
P.Ixz=0;
P.CoG=[0;0;0];

% Constants
P.g=9.80665;

% Move origin to CoG
n=size(P.Surf,2);
for i=1:n
    P.Surf(i).Pos=P.Surf(i).Pos-P.Cog;
end
