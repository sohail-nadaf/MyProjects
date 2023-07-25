% s_to_ss
%   - to initialize state space realization of the dryden transfer
%     functions and propagate the gust model.
classdef s_to_ss < handle
    properties
        num
        den
        A
        B
        C
        D
        gust_state
        Ts
    end
    methods
        function self = s_to_ss(num, den, Ts)
            self.Ts = Ts;
            m = length(num);
            n = length(den);
            self.gust_state = zeros(n,1);
            if den(1) ~= 1
                den = den/den(1);
                num = num/ den(1);
            end
            self.num = num;
            self.den = den;
            %initialize the States of the Controllable Canonical form of
            % the dryden transfer function
            self.A = zeros(n);
            self.B = zeros(n,1);
            self.C = zeros(1,n);
            self.B(n) = 1;
            if m == n
                self.D = num(n);
                for i = 1:n
                    self.A(i,i+1) = 1;
                end
                for i = 1:n
                    self.A(n,i) = -den(n+1-i);
                end
                for i = 1:n
                    self.C(1,i) = num(n+1-i) - den(n+1-i)*num(1);
                end
            else
                self.D = 0;
                for i = 1:n-1
                    self.A(i+1,i) = 1;
                end
                for i = 1:n
                    self.A(1,i) = -den(n+1-i);
                end
                for i = 1:m
                    self.C(1,i) = num(m+1-i);
                end
            end
        end
        function y = update(self, u)
            self.gust_state = self.rk4_step(u);
            y = self.C*self.gust_state + self.D*u;
        end
        function state = rk4_step(self,u)
            F1 = self.f(self.gust_state,u);
            F2 = self.f(self.gust_state + self.Ts/2 * F1,u);
            F3 = self.f(self.gust_state + self.Ts/2 * F2,u);
            F4 = self.f(self.gust_state + self.Ts * F3,u);
            state = self.gust_state + self.Ts/6 * (F1+2*F2+2*F3+F4);
        end
        function xdot = f(self, state, u)
            xdot = self.A * state + u.*self.B;
        end
    end
end
