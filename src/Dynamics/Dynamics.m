function [sys,X0,str,ts,simStateCompliance] = dynamics(t,x,u,flag,P)
    switch flag
        case 0 %initialize
            [sys,X0,str,ts,simStateCompliance]=mdlInitializeSizes(P);
        case 1 %derivatives
            sys=mdlDerivatives(t,x,u,P.mass,P.Ix,P.Iy,P.Iz,P.Ixz);
        case 2 %update
            sys=mdlUpdate(t,x,u);
        case 3 %output
            sys=mdlOutputs(t,x,u);
        case 4 %get time of next var hit
            sys=mdlGetTimeOfNextVarHit(t,x,u);
        case 9 %terminate
            sys=mdlTerminate(t,x,u);
        otherwise
            DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));     
    end
end

function [sys,X0,str,ts,simStateCompliance]=mdlInitializeSizes(P)
    sizes = simsizes;

    sizes.NumContStates  = 12;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 12;
    sizes.NumInputs      = 6;
    sizes.DirFeedthrough = 0;
    sizes.NumSampleTimes = 1;   % at least one sample time is needed

    sys = simsizes(sizes);    
    X0  = P.x0;
    str = [];

    ts  = [0 0];

    simStateCompliance = 'UnknownSimState';
end

function sys=mdlDerivatives(t,x,uu, mass,Jx,Jy,Jz,Jxz)
    pn    = x(1);
    pe    = x(2);
    pd    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    phi   = x(7);
    theta = x(8);
    psi   = x(9);
    p     = x(10);
    q     = x(11);
    r     = x(12);
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    l     = uu(4);
    m     = uu(5);
    n     = uu(6);    
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
    
    R=roll*pitch*yaw;
    res=R'*[u;v;w];
    pndot = res(1);    
    pedot = res(2); 
    pddot = res(3);
    
    T=[
        1   sin(phi)*tan(theta) cos(phi)*tan(theta);
        0   cos(phi)            -sin(phi);
        0   sin(phi)*sec(theta) cos(phi)*sec(theta)
    ];
    res=T*[p;q;r];
    phidot = res(1);    
    thetadot = res(2);    
    psidot = res(3);
    
    res=[r*v-q*w;p*w-r*u;q*u-p*v]+[fx;fy;fz]/mass;
    
    
    udot = res(1);    
    vdot = res(2);    
    wdot = res(3);
    
    J=[
        Jx 0 -Jxz;
        0 Jy 0;
        -Jxz 0 Jz
    ];
    
    T2=[
        0 r -q;
        -r 0 p;
        q -p 0
    ];

    res=inv(J)*(T2*J*[p;q;r]+[l;m;n]);    
    pdot = res(1);    
    qdot = res(2);    
    rdot = res(3); 

    sys = [pndot; pedot; pddot; udot; vdot; wdot; phidot; thetadot;...
        psidot; pdot; qdot; rdot];
end

function sys=mdlUpdate(t,x,u)
    sys=[];
end

function sys=mdlOutputs(t,x,u)
    sys=x;
end

function sys=mdlGetTimeOfNextVarHit(t,x,u)
    sampleTime=1;
    sys=t+sampleTime;
end

function sys=mdlTerminate(t,x,u)
    sys=[];
end