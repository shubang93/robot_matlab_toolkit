classdef RobotDef < handle
   properties
       links
       num_links
 
   end
   methods (Access=private)
        
   end
   methods (Access=public)
       function robotDef = RobotDef(linkDefs)
          assert(isa(linkDefs(1), 'LinkDef'), 'Arguments are not of type LinkDef');
          robotDef.links = linkDefs;
          robotDef.num_links =length(linkDefs);
       end
       function [] = updateJointState(obj, jointStates) 
           assert(length(jointStates)==obj.num_links, 'Number of joints dont match number of links');
            for i=1:obj.num_links
               obj.links(i).update(jointStates(i));
            end
       end
       function T = fwdKinematics(obj, jointStates)
          assert(length(jointStates)==obj.num_links, 'Number of joints do not match robot joints');
          obj.updateJointState(jointStates);
          T = makehgtform;
          for i=1:obj.num_links
              T = T*obj.links(i).linkTransform();
          end
       end
       
       function JointAngles = invKinematics_3(obj, goalState)
          assert(obj.num_links==3, 'Need number of robot links to be 3');
          for i=1:obj.num_links
              assert(obj.links(i).type==0, 'All links must be revolute type');
          end
          JointAngles = zeros(1, 3);
          x = goalState(1, 4);
          y = goalState(2, 4);
          z = goalState(3, 4);
          l = zeros(1, 3);
          for i=1:3
             if(obj.links(i).alpha==0 || obj.links(i).alpha==pi || obj.links(i).alpha==-pi)
                 l(i) = obj.links(i).a;
             else
                 l(i) = obj.links(i).d;
             end
          end
          
          JointAngles(1) = atan2(y, x);
          
          c3 = (x^2+y^2+(z-l(1))^2-l(2)^2-l(3)^2)/(2*l(2)*l(3));
          s3 = sqrt(1-c3^2);
          if isreal(s3)
            JointAngles(3) = atan2(s3, c3);
          else
              error('Point not reachable by robot');
          end
          
          k1 = l(2) + l(3)*c3;
          k2 = l(3)*s3;
          d = sqrt(k1^2+k2^2);
          gamma = atan2(k2, k1);
          h = z-l(1);
          r = sqrt(x^2+y^2);
          
          JointAngles(2) = atan2(h,r)+gamma;
          for i=1:3
              if JointAngles(i)>pi || JointAngles(i) < 0
                  error('Point out of robot reach')
              end
          end
       end
       
       function jacob3 = jacobian_3dof(obj, theta1, theta2, theta3)
           l = zeros(1,3);
          for i=1:3
             if(obj.links(i).alpha==0 || obj.links(i).alpha==pi || obj.links(i).alpha==-pi)
                 l(i) = obj.links(i).a;
             else
                 l(i) = obj.links(i).d;
             end
          end
          jacob3 = [ -sin(theta1)*(l(2)*cos(theta2) + l(3)*cos(theta2 - theta3)), -cos(theta1)*(l(2)*sin(theta2) + l(3)*sin(theta2 - theta3)), l(3)*sin(theta2 - theta3)*cos(theta1);...
                      cos(theta1)*(l(2)*cos(theta2) + l(3)*cos(theta2 - theta3)), -sin(theta1)*(l(2)*sin(theta2) + l(3)*sin(theta2 - theta3)), l(3)*sin(theta2 - theta3)*sin(theta1);...
                                                                           0,               l(2)*cos(theta2) + l(3)*cos(theta2 - theta3),            -l(3)*cos(theta2 - theta3)];
 
       end
       
       function joint_state = currentJointState(obj)
           joint_state = zeros(1, obj.num_links);
          for i=1:obj.num_links
              joint_state(i) = obj.links(i).currentState();
          end
       end
   end
end