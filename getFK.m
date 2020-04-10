function T = getFK(DH_table, start_row, end_row)

T = eye(4);

for i = start_row : end_row
    % target: i
    
    a     = DH_table(i,1);
    % Link length, distance between Zi-1 and Zi, measured along Xi
    
    alpha = DH_table(i,2);
    % Link twist, angle between Zi-1 and Zi, measured in the plane normal to Xi (RHR)
    
    d     = DH_table(i,3);
    % Link offset, distanc between Xi-1 and Xi, measured along Zi-1
    
    th    = DH_table(i,4);
    % Joint Angle, angle between Xi-1 and Xi, measured in the plane normal to Zi-1 (RHR)
    
    A = [  cos(th),  -sin(th)*cos(alpha),  sin(th)*sin(alpha),  a*cos(th);...
           sin(th),   cos(th)*cos(alpha), -cos(th)*sin(alpha),  a*sin(th); ...
                 0,           sin(alpha),          cos(alpha),          d; ...
                 0,                    0,                   0,          1];
   
    T = T * A;
end

end
