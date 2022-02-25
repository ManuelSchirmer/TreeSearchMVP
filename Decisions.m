classdef Decisions
% Mock-up decisions
    
    methods(Static)
        function decisions = calculateDecisions_Ego()
            decisions{1} = 'Ego_A';
            decisions{2} = 'Ego_B';
            decisions{3} = 'Ego_C';
            decisions{4} = 'Ego_D';
            decisions{5} = 'Ego_E';
            decisions{6} = 'Ego_F';
            decisions{7} = 'Ego_G';
            decisions{8} = 'Ego_H';
        end
        
        function decisions = calculateDecisions_Other()
            decisions{1} = 'Other_{V1}';
            decisions{2} = 'Other_{V2}';
        end
    end
end

