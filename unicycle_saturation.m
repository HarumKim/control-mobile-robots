function [v,w] = unicycle_saturation(wrmax,wlmax,v,w,r,d)
% Applies saturation to v and w using the maximum wheel speeds
% of the right and left wheels (wrmax, wlmax).

    % Saturation on w from wheel limits
    if w > 2*wrmax*r/d
        w = 2*wrmax*r/d;
    end
    if w < -(2*wrmax*r/d)
        w = -2*wrmax*r/d;
    end
    
    % Saturation on v (right and left wheel conditions)
    if v > r*wrmax - d/2*w
        v = r*wrmax - d/2*w;
    end
    if v > r*wlmax + d/2*w
        v = r*wlmax + d/2*w;
    end
    
    if v < -(r*wrmax - d/2*w)
        v = -(r*wrmax - d/2*w);
    end
    if v < -(r*wlmax + d/2*w)
        v = -(r*wlmax + d/2*w);
    end
end
