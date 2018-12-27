function [F M] = GetForces(P,u)
    t=u(1);
    state=u(2:13);
    ctrl=u(14:size(u,1));
    
end