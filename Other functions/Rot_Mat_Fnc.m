% This function calculates the rotation matrix for
% rotation from tangent frame to body frame.

function C = Rot_Mat_Fnc(ang)

cr=cos(ang(1));
sr=sin(ang(1));

cp=cos(ang(2));
sp=sin(ang(2));

cy=cos(ang(3));
sy=sin(ang(3));

% Se eq. (2.14) in Groves (2008).
C=[cy*cp sy*cp -sp; 
    -sy*cr+cy*sp*sr cy*cr+sy*sp*sr cp*sr; 
    sy*sr+cy*sp*cr -cy*sr+sy*sp*cr cp*cr];

