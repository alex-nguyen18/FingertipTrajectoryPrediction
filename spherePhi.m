function phi = spherePhi(y,rS,rC)
    phi = acos(rC/rS);
    phi = pi - phi;
    
    return phi
end