function DrawPlane(state,ctrl)
    pn=state(1);

    
end

function [V,F,C]=planeModel(P)
    n=size(P.surf,2);
    V=[];
    for i = 1:n
        sp=[
            P.surf(i).c/4,-P.surf(i).b/2,0;
            P.surf(i).c/4,P.surf(i).b/2,0;
            -3*P.surf(i).c/4,P.surf(i).b/2,0;
            -3*P.surf(i).c/4,-P.surf(i).b/2,0;
        ];
        sp=(Rotate(P.surf(i).Rot)*sp')';
        sp=Translate(sp,P.surf(i).Pos);
        V=[V;sp];
    end
end