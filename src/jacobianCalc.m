syms q1 q2 q3

A1 = [cos(q1) 0 -sin(q1) 0 ; sin(q1) 0 cos(q1) 0 ; 0 1 0 95 ; 0 0 0 1];
A2 = [sin(q2) cos(q2) 0 100*sin(q2) ; -cos(q2) sin(q2) 0 -100*cos(q2) ; 0 0 1 0; 0 0 0 1];
A3 = [-sin(q3) -cos(q3) 0 -100*sin(q3) ; cos(q3) -sin(q3) 0 100*cos(q3) ; 0 0 1 0 ; 0 0 0 1];
T1 = A1;
T2 = A1*A2;
T3 = A1*A2*A3;

z0 = [0;0;1];
z1 = T1(1:3, 3);
z2 = T2(1:3, 3);
z3 = T3(1:3, 3);

o0 = [0;0;00];
o1 = T1(1:3, 4);
o2 = T2(1:3, 4);
o3 = T3(1:3, 4);

jacob = [cross(z0, o3 - o0) cross(z1, o3 - o1) cross(z2, o3 - o2) ; z0 z1 z2]
