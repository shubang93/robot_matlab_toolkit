classdef LinkDef < handle
    properties
        theta
        alpha
        a
        d
        type
    end
    methods
        function link = LinkDef(values)
            [m, n] = size(values);
            assert(m==1 && n==5, 'Vector passed in is not 1x5');
           link.theta = values(1);
           link.d = values(2);
           link.a = values(3);
           link.alpha = values(4);
           link.type = values(5);
        end
        function [] = update(obj, val)
           if obj.type==0
               obj.theta = val;
           else
               obj.d = val
           end
        end
        function T = linkTransform(obj)
           T = tf.rotz_t(obj.theta)*tf.translate(0, 0, obj.d)*tf.translate(obj.a, 0, 0)*tf.rotx_t(obj.alpha); 
        end
        function q = currentJointState(obj)
            if obj.type==0
                q = obj.theta;
            else
                q = obj.d;
        end
    end
end

