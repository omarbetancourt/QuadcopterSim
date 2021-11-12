classdef Target < handle
    properties
        pos
        isMapped
    end
    
    methods
        function obj = Target(initPos)
            obj.pos = initPos;
            obj.isMapped = false;
        end
        
        function position = GetPos(obj)
            position = obj.pos;
        end
        
        function obj = TestCollision(obj, swarm_mem)
            if swarm_mem.GetDmt <= 2
                obj.isMapped = true;
            end
        end
        
        function mapped = IsMapped(obj)
            mapped = obj.isMapped;
        end
    end
end