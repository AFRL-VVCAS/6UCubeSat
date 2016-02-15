function x_cross=skew(x)
% Converts a 3 by 1 vector into a skew-symmetric matrix
% Updated: 22 Apil 2015
% Author: Kerianne Gross; kerianne.gross@us.af.mil
% Adapted from Dr. Eric Swenson's Model (at AFIT)

% Check for correct size
if max(size(x))~=3 || min(size(x))~= 1
    disp('not a 3by1 vector')
    return
end

% Create skew symmetric-matrix
x_cross = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];

end
