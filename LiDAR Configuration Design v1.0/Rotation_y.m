function R_y = Rotation_y(theta)
    R_y = [cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
end