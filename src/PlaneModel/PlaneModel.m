% Surface
% Right wing
P.Surf(1).b=1;          % Span
P.Surf(1).c=0.3;        % Chord
P.Surf(1).m=1;          % Mass
P.Surf(1).pos=[0;0;0];  % Position
P.Surf(1).Rot=[0;0;0];  % Rotation
P.Surf(1).I=GetInertia(P.Surf(1));  % Inertia

% Left wing
P.Surf(2).b=1;          % Span
P.Surf(2).c=0.3;        % Chord
P.Surf(2).m=1;          % Mass
P.Surf(2).pos=[0;0;0];  % Position
P.Surf(2).I=GetInertia(P.Surf(2));  % Inertia

% HTail
P.Surf(3).b=1;          % Span
P.Surf(3).c=0.3;        % Chord
P.Surf(3).m=1;          % Mass
P.Surf(3).pos=[0;0;0];  % Position
P.Surf(3).I=GetInertia(P.Surf(3));  % Inertia

% VTail
P.Surf(4).b=1;          % Span
P.Surf(4).c=0.3;        % Chord
P.Surf(4).m=1;          % Mass
P.Surf(4).pos=[0;0;0];  % Position
P.Surf(4).I=GetInertia(P.Surf(4));  % Inertia

P.mass=0;
