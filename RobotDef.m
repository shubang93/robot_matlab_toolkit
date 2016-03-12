classdef RobotDef < handle
   properties
       links
       num_links
   end
   methods (Access=private)
        function [] = updateJointState(obj, jointStates) 
            for i=1:obj.num_links
               obj.links(i).update(jointStates(i));
            end
       end
   end
   methods (Access=public)
       function robotDef = RobotDef(linkDefs)
%           assert(linkDefs(1).IsOfClass('LinkDef')==0, 'Class type for link does not match');
          robotDef.links = linkDefs;
          robotDef.num_links =length(linkDefs);
       end
       
       function T = fwdKinematics(obj, jointStates)
          assert(length(jointStates)==obj.num_links, 'Number of joints do not match robot joints');
          obj.updateJointState(jointStates);
          T = makehgtform;
          for i=1:obj.num_links
              T = T*obj.links(i).linkTransform();
          end
       end
   end
end