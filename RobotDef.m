classdef RobotDef
   properties
       links
       num_links
   end
   methods (Access=private)
        function [] = updateJointState(jointStates) 
            for i=1:obj.num_links
               if obj.links(i).type==0 
                   obj.links(i).theta = jointStates(i);
               else
                   obj.links(i).d = jointStates(i);
               end
            end
       end
   end
   methods (Access=public)
       function robotDef = RobotDef(linkDefs)
%           assert(class(LinkDef) == class(linkDefs(1)), 'Argument has to be of type LinkDef');
          robotDef.links = linkDefs;
          robotDef.num_links =length(linkDefs);
       end
       
       function T = fwdKinematics(jointStates)
          assert(length(joinStates)==obj.num_links, 'Number of joints do not match robot joints');
          obj.updateJointState(jointStates);
          T = makehgtform;
          for i=1:obj.num_links
              T = T*obj.links(i).linkTransform();
          end
       end
   end
end