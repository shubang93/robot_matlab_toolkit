classdef LinkDef
    properties
        theta
        alpha
        a
        d
        type
    end
    methods
        function link = Link(values)
            [m, n] = size(values);
            assert(m==1 & n==5, 'Vector passed in is not 1x5');
            if m==1 & n==5
               link.theta = values(1);
               link.d = values(2);
               link.a = values(3);
               link.alpha = values(4);
               link.type = values(5);
            else
                error('Need to pass in a 5 column vector for DH values');
            end
        end
    end
end

