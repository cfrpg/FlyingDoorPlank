function out = Sensor(P,state)
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

    Va=norm([u,v,w]);
    alt=-pd;
   

    out=[Va;alt];
end