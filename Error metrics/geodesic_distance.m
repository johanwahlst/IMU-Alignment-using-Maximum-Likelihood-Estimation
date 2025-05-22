function phi = geodesic_distance(R1, R2)

    R_rel = R1' * R2;
    phi = acos((trace(R_rel) - 1) / 2);
    phi = real(phi); 
    
end