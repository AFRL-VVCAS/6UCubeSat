function x_cross=skew(x)
%Converts a 3 by 1 vector into a skew-symmetric matrix

% Check for correct size

if max(size(x))~=3 || min(size(x))~= 1
    disp('not a 3by1 vector')
    return
end

x_cross = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];

end
%eof