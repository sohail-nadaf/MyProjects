% wind Simulation - simulates steady state and wind gusts
%
% mavMatSim 
%     - Beard & McLain, PUP, 2012
%     - Update history:  
%         12/27/2018 - RWB
classdef wind_simulation < handle
   %--------------------------------
    properties
        steady_state
        gust_b_u
        gust_b_v
        gust_b_w
        gust
        Ts
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = wind_simulation(Ts,dryd)
            self.Ts = Ts;
            self.steady_state = [0; 0; 0];
            self.gust_b_u = s_to_ss(dryd.Unum, dryd.Uden, Ts);
            self.gust_b_v = s_to_ss(dryd.Vnum, dryd.Vden, Ts);
            self.gust_b_w = s_to_ss(dryd.Wnum, dryd.Wden, Ts);
            self.gust = [0; 0; 0];            
        end
        %---------------------------
        function y = update(self, steadwind,Va)
            dryd = dryden(Va);
            self.gust_b_u = s_to_ss(dryd.Unum, dryd.Uden, self.Ts);
            self.gust_b_v = s_to_ss(dryd.Vnum, dryd.Vden, self.Ts);
            self.gust_b_w = s_to_ss(dryd.Wnum, dryd.Wden, self.Ts);
            w = randn;
            self.steady_state = steadwind;
            self.gust = [self.gust_b_u.update(w);...
                self.gust_b_v.update(w);...
                self.gust_b_w.update(w)];
            y = self.steady_state + self.gust;% the ss wind needs to be rotated to body frame
        end
        %---------------------------- 
    end
end