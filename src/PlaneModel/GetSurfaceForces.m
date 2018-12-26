function [F M] = GetSurfaceForces(S,V,rho)
    Va=sqrt(V(1)*V(1)+V(3)*V(3));
    alpha=atan2(V(3),V(1));
    q=0.5*rho*Va*Va*S.b*S.c;
    F=[0;0;0];
    M=[0;0;0];
    alpha_p=alpha;
    cma=0;  % No pitch moment @ small AoA
    if alpha_p>pi/2
        alpha_p=alpha_p-pi;
        cma=S.Cma;
    elseif alpha_p<-pi/2
        alpha_p=alpha_p+pi;
        cma=S.Cma;
    end

    s=sigma(alpha,S.a0,S.M);
    CL=(1-s)*S.CLa*alpha_p+s*sin(2*alpha);
    CD=(1-s)*(CL*CL)/(pi*S.e*S.AR))+s*2*sin(alpha)^2;
    Cm=(1-s)*cma*alpha_p+s*(-0.5*sin(alpha)*(0.7*cos(alpha)+1));
    
    % Drag
    F(1)=-q*CD;
    % Side force
    F(2)=0;
    % Lift
    F(3)=-q*CL;

    % Roll moment
    M(1)=0;
    % Pitch moment
    M(2)=q*Cm;
    % Yaw moment
    M(3)=0;
end

function s = sigma(a,a0,m)
    s=_sigma(a,a0,m)*_sigma(pi-a,a0,m)*_sigma(pi+a,a0,m);
end

function s = _sigma(a,a0,m)
    s=(1+exp(-m*(a-a0) ) + exp(m*(a+a0)) ) / ((1 + exp(-m*(a-a0))) * (1+exp(m*(a+a0))));
end