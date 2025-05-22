function Rs = Octahedral_Group

R1 = Rot_Mat_Fnc([pi/2 0 0]);
R2 = Rot_Mat_Fnc([0 pi/2 0]);
R3 = Rot_Mat_Fnc([0 0 pi/2]);
Rs = zeros(3,3,24);
for i = 1:4
    for j = 1:4
        Rs(:,:,i+4*(j-1)) = R3^(j)*R1^(i);
    end
end
for i = 1:4
    Rs(:,:,16+i) = R3^(i)*R2;
end
for i = 1:4
    Rs(:,:,20+i) = R3^(i)*R2';
end

end