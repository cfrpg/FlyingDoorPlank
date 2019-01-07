function out = GetForces(P,u)
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

    F=[0;0;0];
    M=[0;0;0];

    n=size(ctrl,1);
    for i = 1:n
        Va=[-u;-v;-w];
        Va=(Rotate(P.Surf(i).Rot)*Rotate([0,-ctrl(i+1),0]))'*Va;
        [Fi Mi]=GetSurfaceForce(P.Surf(i),Va,P.rho);
        Fi=Rotate(P.Surf(i).Rot)*Rotate([0,-ctrl(i+1),0])*Fi;
        Mi=Rotate(P.Surf(i).Rot)*Rotate([0,-ctrl(i+1),0])*Mi;
        Mi=Mi+cross(Fi,P.Surf(i).Pos);
        F=F+Fi;
        M=M+Mi;
    end
    out=[F;M];
end
