function out = GetForces2(P,u)
    t=u(1);
    state=u(2:13);
    ctrl=u(14:P.SurfCnt+14);
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
    Va=norm([u,v,w]);
    %Gravity
    F=Rotate([phi;theta;psi])*[0;0;P.mass*g];
    %Propulsion force
    F(1)=F(1)+0.5*P.rho*P.Sprop*P.Cprop*((P.Kmotor*ctrl(1))^2);
    du=P.Kmotor*ctrl(1)-u;
    
    %Propeller torque
    M=[-P.Ktp*(P.Ko*ctrl(1))^2;0;0];
    
    n=P.SurfCnt;    
    for i = 1:n
        Vel=[u;v;w];%-cross([p;q;r],P.Surf(i).Pos);
        T=(Rotate(-P.Surf(i).Rot)*Rotate([0,-ctrl(i+1),0]));
        Vel=T'*Vel;
        [Fi,Mi]=GetSurfaceForces(P.Surf(i),Vel,P.rho);       
        Fi=T*Fi;
        Mi=T*Mi;
        Mi=Mi+cross(Fi,P.Surf(i).Pos);
        F=F+Fi;
        %M=M+Mi;
    end

    out=[F;M];
end
