function DrawPlane(P,u)
    t=u(1);
    state=u(2:13);
    ctrl=u(14:size(u,1));
    pn=state(1);
    pe=state(2);
    pd=state(3);
    phi=state(7);
    theta=state(8);
    psi=state(9);

    fs=1000;
    ps=300;
    persistent hplane;    
    R=[        
        0 1 0;
        1 0 0;
        0 0 -1;
        ];
    [V,F,C]=planeModel(P,ctrl);
    V=Rotate([phi;theta;psi])'*V'*ps;
    V=Translate(V,[pn;pe;pd]);
    V=R*V;
    
    if(t<0.1)
        figure(1),clf
        hplane=patch('Vertices',V','Faces',F,'FaceVertexCData',C,'FaceColor','flat');        
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(45,45)
        axis([-1,1,-1,1,-1,1]*fs);
        grid on
        hold on
    else
        set(hplane,'Vertices',V');
    end
end

function [V,F,C]=planeModel(P,ctrl)
    n=size(P.Surf,2);
    V=[];
    F=zeros(n,4);
    C=zeros(n,3);
    for i = 1:n
        sp=[
            P.Surf(i).c/4,-P.Surf(i).b/2,0;
            P.Surf(i).c/4,P.Surf(i).b/2,0;
            -3*P.Surf(i).c/4,P.Surf(i).b/2,0;
            -3*P.Surf(i).c/4,-P.Surf(i).b/2,0;
        ];
        sp=(Rotate(P.Surf(i).Rot)*Rotate([0,-ctrl(i+1),0])*sp')';
        sp=Translate(sp',P.Surf(i).Pos)';
        V=[V;sp];
        F(i,:)=[4*i-3,4*i-2,4*i-1,4*i];
        C(i,:)=[0,1,0];
    end
end