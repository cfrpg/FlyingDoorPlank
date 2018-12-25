function V = Translate(V,pos)
    V=V+repmat(pos,1,size(V,2));
end