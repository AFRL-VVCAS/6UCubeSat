%Function R2Q
%Eigenaxis, Euler Angle, & Euler parameter function
%Input: Rotation Matrix
%Output: eigenaxis, principle Euler angle, and quaternions

function [a,phi,q] = R2Q(R,v)

if (nargin == 1)
    
    tr=trace(R);
     if abs(sum(tr)-3) < eps
        a = [0 0 1];
        phi = 0;
         q = [0 0 0 1]';
     else
        phi=acos((1/2)*(tr-1));
        ax=(1/(2*sin(phi)))*(R'-R);
        a=[ax(3,2);ax(1,3);ax(2,1)];
        q4=cos(phi/2);
        qu=sin(phi/2)*a;
        q=[qu;q4];
    end
end

if (nargin == 2)
    
    if (v =='deg')
        
        tr=trace(R);
        if abs(sum(tr)-3) < eps
            a = [0 0 1];
            phi = 0;
            q = [0 0 0 1];
        else
            phi=acosd((1/2)*(tr-1));
            ax=(1/(2*sind(phi)))*(R'-R);
            a=[ax(3,2);ax(1,3);ax(2,1)];
            q4=cosd(phi/2);
            qu=sind(phi/2)*a;
            q=[qu;q4];
        end
    end
end