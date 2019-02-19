function R=Rotate(rot)
    phi=rot(1);
    theta=rot(2);
    psi=rot(3);

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
    %R=yaw*pitch*roll;
    R=roll*pitch*yaw;
end