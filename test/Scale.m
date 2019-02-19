function out = Scale(P,u)
    t=u(1);
    state=u(2:13);
    ctrl=u(14:size(u,1));
    pn    = state(1);
    pe    = state(2);
    pd    = state(3);
    u     = state(4);
    v     = state(5);
    w     = state(6);
    phi   = state(7);
    theta = state(8);
    psi   = state(9);
    p     = state(10);
    q     = state(11);
    r     = state(12);

    g=9.80665;
    
    roll=[
        1 0 0;
        0 cos(phi) sin(phi);
        0 -sin(phi) cos(phi)];
    pitch=[
        cos(theta), 0, -sin(theta);
        0, 1, 0;
        sin(theta), 0, cos(theta)];
    yaw=[
        cos(psi), sin(psi), 0;
        -sin(psi), cos(psi), 0;
        0, 0, 1];    
    Rvb=roll*pitch*yaw;
   
    %F=Rvb*[0;0;P.mass*g];
    F=[0;0;0];
    M=[0;0;0];
    
    n=size(ctrl,1)-1;
    Fs=zeros(n,3);
    Ms=zeros(n,3);
    Fs2=zeros(n,3);
    Ms2=zeros(n,3);
    u=10*cos(theta);
    v=0;
    w=10*sin(theta);
%     for i = 1:n
%         Va=[u;v;w];
%         Va=(Rotate(-P.Surf(i).Rot)*Rotate([0,-ctrl(i+1),0]))'*Va;
%         [Fi,Mi]=GetSurfaceForces(P.Surf(i),Va,P.rho);
%         Fs(i,:)=Fi;
%         Ms(i,:)=Mi;
%         %Fi=Rotate(P.Surf(i).Rot)*Rotate([0,-ctrl(i+1),0])*Fi;
%         %Mi=Rotate(P.Surf(i).Rot)*Rotate([0,-ctrl(i+1),0])*Mi;
%         Fi=Rotate(-P.Surf(i).Rot)*Rotate([0,-ctrl(i+1),0])*Fi;
%         Mi=Rotate(-P.Surf(i).Rot)*Rotate([0,-ctrl(i+1),0])*Mi;
%         Mi=Mi+cross(Fi,P.Surf(i).Pos);
%         Fs2(i,:)=Fi;
%         Ms2(i,:)=Mi;
%         F=F+Fi;
%         M=M+Mi;
%     end
    alpha=-theta;
    T1=[cos(alpha),-sin(alpha);sin(alpha),cos(alpha)];
    i=1;
    Va=[10;0;0];
    T=(Rotate(-P.Surf(i).Rot)*Rotate([0,-ctrl(i+1),0]));
    Va=T'*Va;
    [Fi,Mi]=GetSurfaceForces(P.Surf(i),Va,P.rho);
    Fs(i,:)=Fi;
    Ms(i,:)=Mi;    
    Fi=T*Fi;
    Mi=T*Mi;
    Mi=Mi+cross(Fi,P.Surf(i).Pos);
    Fs2(i,:)=Fi;
    Ms2(i,:)=Mi;
    F=F+Fi;
    M=M+Mi;
    
    i=2;
    Va=[u;v;w];
    T=(Rotate(-P.Surf(i).Rot)*Rotate([0,-ctrl(i+1),0]));
    Va=T'*Va;
    [Fi,Mi]=GetSurfaceForces(P.Surf(i),Va,P.rho);
    Fs(i,:)=Fi;
    Ms(i,:)=Mi;   
    Fi=T*Fi;
    Mi=T*Mi;
    
    Fxz=[cos(alpha),-sin(alpha);sin(alpha),cos(alpha)]*[Fi(1);Fi(3)];
    Fi(1)=Fxz(1);
    Fi(3)=Fxz(2);   
    Fs2(i,:)=Fi;
    Ms2(i,:)=Mi;
    F=F+Fi;
    M=M+Mi;
    %gravity
    D1=[0,0,0];
    out=[Fs2;Ms2;D1;D1];   
end
