% mav dynamics - implement rigid body dynamics for mav

classdef mav_dynamics < handle
   %--------------------------------
    properties
        ts_simulation
        state
        Va
        alpha
        beta
        wind % NED 
        true_state % state inclusive of all models
    end
    %--------------------------------
    methods
        %------initialize-----------
        function self = mav_dynamics(Ts, MAV)
            self.ts_simulation = Ts; % time step
            self.state = [MAV.pn0; MAV.pe0; MAV.pd0; MAV.u0; MAV.v0; MAV.w0;...
                          MAV.e0; MAV.e1; MAV.e2; MAV.e3; MAV.p0; MAV.q0; MAV.r0];
            self.Va = MAV.u0;
            self.alpha = 0;
            self.beta = 0;
            self.wind = [0; 0; 0];
            addpath('../message_types'); self.true_state = msg_state();
        end
        %---------------------------
        function self=update_state(self, delta, wind, MAV)
            %
            % Integrate the differential equations defining dynamics
            % forces_moments are the forces and moments on the MAV.
            % 
            
            % get forces and moments acting on rigid body
            forces_moments = self.forces_moments_fun(delta, MAV, self.state);
            
            % Integrate ODE using Runge-Kutta RK4 algorithm
            k1 = self.derivatives(self.state, forces_moments, MAV);
            k2 = self.derivatives(self.state + self.ts_simulation/2*k1, forces_moments, MAV);
            k3 = self.derivatives(self.state + self.ts_simulation/2*k2, forces_moments, MAV);
            k4 = self.derivatives(self.state + self.ts_simulation*k3, forces_moments, MAV);
            self.state = self.state + self.ts_simulation/6 * (k1 + 2*k2 + 2*k3 + k4);
            
            % quaternion normalization
            self.state(7:10) = self.state(7:10)/norm(self.state(7:10));
            
            % update the airspeed, angle of attack, and side slip angles
            self.update_velocity_data(wind);
            
            % update the message class for the true state
            self.update_true_state();
        end
        %----------------------------
        function xdot = derivatives(self, state, forces_moments, MAV)
            % states to propagate rigid body dynamics
            pn    = state(1);
            pe    = state(2);
            pd    = state(3);
            u     = state(4);
            v     = state(5);
            w     = state(6);
            e0    = state(7);
            e1    = state(8);
            e2    = state(9);
            e3    = state(10);
            p     = state(11);
            q     = state(12);
            r     = state(13);

            %Forces Moments
            fx    = forces_moments(1);
            fy    = forces_moments(2);
            fz    = forces_moments(3);
            ell   = forces_moments(4);
            m     = forces_moments(5);
            n     = forces_moments(6);

            % position kinematics
            pn_dot = (((e0^2)+(e1^2)-(e2^2)-(e3^2))*u) + (2*((e1*e2) - (e0*e3))*v) + (2*((e1*e3) + (e0*e2))*w);
            pe_dot = (2*((e1*e2) + (e0*e3))*u) + (((e0^2)-(e1^2)+(e2^2)-(e3^3))*v) + (2*((e2*e3) - (e0*e1))*w);
            pd_dot = (2*((e1*e3) - (e0*e2))*u) + (2*((e2*e3) + (e0*e1))*v) + (((e0^2)-(e1^2)-(e2^2)+(e3^2))*w);

            % position dynamics
            u_dot = (r * v - q * w) + fx/MAV.mass;
            v_dot = (p * w - r * u) + fy/MAV.mass;
            w_dot = (q * u - p * v) + fz/MAV.mass;
            
            % rotational kinematics
            e0_dot = 0.5 *(-p * e1 - q * e2 - r * e3);
            e1_dot = 0.5 * (p * e0 + r * e2 - q * e3);
            e2_dot = 0.5 * (q * e0 - r * e1 + p * e3);
            e3_dot = 0.5 * (r * e0 + q * e1 - p * e2);
            
            % rotational dynamics
            p_dot = MAV.Gamma1 * p * q - MAV.Gamma2 * q * r + MAV.Gamma3 * ell + MAV.Gamma4 * n;
            q_dot = MAV.Gamma5 * p * r - MAV.Gamma6 * (p^2 - r^2) + m/MAV.Jy;
            r_dot = MAV.Gamma7 * p * q - MAV.Gamma1 * q * r + MAV.Gamma4 * ell + MAV.Gamma8 * n;

            % return the derivatives
            xdot = [pn_dot; pe_dot; pd_dot; u_dot; v_dot; w_dot;...
                    e0_dot; e1_dot; e2_dot; e3_dot; p_dot; q_dot; r_dot];
        end
        %----------------------------
        function self=update_velocity_data(self, wind)
            % copy wind to the property of mav dynamics
            self.wind = wind;

            % calculate airspeed in body frame from the wind in body frame
            arsp_b = [self.state(4) - wind(1);...
                      self.state(5) - wind(2);...
                      self.state(6) - wind(3)];
            
            % update airspeed, alpha and beta in stability frame
            self.Va = sqrt(arsp_b(1)^2 + arsp_b(2)^2 + arsp_b(3)^2);
            self.alpha = atan(arsp_b(3) / arsp_b(1));
            self.beta = asin(arsp_b(2) / self.Va);
        end
        %----------------------------
        function out = forces_moments_fun(self, delta, MAV, state)
            % copy the commanded throttle and control surface deflections
            da = delta(1);
            de = delta(2);
            dt = delta(3);
            dr = delta(4);

            % copy quaternion coefficients
            e0 = self.state(7);
            e1 = self.state(8);
            e2 = self.state(9);
            e3 = self.state(10);

            % phi = self.true_state.phi;
            % theta = self.true_state.theta;
            % psi = self.true_state.psi;

            p     = state(11);
            q     = state(12);
            r     = state(13);

            %
            Va = self.Va;
            alpha = self.alpha;
            beta = self.beta;
            
            % gravity model
            %------------------------------------------------------------------------------------------------------------------
            % quaternion representation of gravity in body frame
            gravity_force = MAV.mass*MAV.gravity.*[2 * (e1 * e3 - e2 * e0);...
                                                   2 * (e2 * e3 - e1 * e0);...
                                                   e3^2 + e0^2 - e1^2 - e2^2];

            % euler angle implementation of gravity model
            % gravity_force = [-MAV.mass * MAV.gravity * theta;...
            %                 MAV.mass * MAV.gravity * cos(theta)*sin(phi);...
            %                 MAV.mass * MAV.gravity * cos(theta)*cos(phi)];
            %------------------------------------------------------------------------------------------------------------------

            % aerodynamic model
            %------------------------------------------------------------------------------------------------------------------
            % sigmoid function for incorporating stall behaviour
            sigmoidf        = (1 + exp(1)^(-MAV.M * (alpha - MAV.alpha0)) + exp(1)^(MAV.M * (alpha + MAV.alpha0)))/...
                              (((1 + exp(1)^(-MAV.M * (alpha - MAV.alpha0)))) * ((1 + exp(1)^(MAV.M * (alpha + MAV.alpha0)))));
                    
            % compute aerodynamic coefficients:
            % X_alpha indicates function
            % of alpha ie X(alpha)
            CL_alpha        = (1-sigmoidf) * (MAV.C_L_0 + MAV.C_L_alpha * alpha)...
                              +sigmoidf * (2 * sign(alpha) * (sin(alpha))^2 *  cos (alpha));%sign(alpha) = 1, 0 -1 based on sign of alpha
            CD_alpha        = MAV.C_D_p + (MAV.C_L_0 + MAV.C_L_alpha * alpha)^2/...
                              (pi * MAV.e * MAV.AR);

            CX_alpha        = -CD_alpha * cos(alpha) + CL_alpha * sin(alpha);
            CXq_alpha       = -MAV.C_D_q * cos(alpha) + MAV.C_L_q * sin(alpha);
            CXdeltae_alpha  = -MAV.C_D_delta_e * cos(alpha) + MAV.C_L_delta_e * sin(alpha);
            CZ_alpha        = -CD_alpha * sin(alpha) - CL_alpha * cos(alpha);
            CZq_alpha       = -MAV.C_D_q * sin(alpha) - MAV.C_L_q * cos(alpha);
            CZdeltae_alpha  = -MAV.C_D_delta_e * sin(alpha) - MAV.C_L_delta_e * cos(alpha);
            
            
            if Va == 0 % avoid division by zero
                aerodynamic_force = [0;0;0];
                aerodynamic_torque = [0;0;0];
            else
                aerodynamic_force  = (0.5 * MAV.rho * (Va)^2 * MAV.S_wing).*[CX_alpha + CXq_alpha * MAV.c * q/(2 * Va);...
                    (MAV.C_Y_0 + MAV.C_Y_beta * beta )+( MAV.C_Y_p * MAV.b * p + MAV.C_Y_r * MAV.b *r)/(2 * Va);...
                    (CZ_alpha) + (CZq_alpha * MAV.c * q)/(2 * Va)];
                
                % the following does not compute,  error in vertcat.
                %-----------------------------------------------------------------------------------------------------------
                % aerodynamic_force  = (0.5 * MAV.rho * (Va)^2 * MAV.S_wing).*[CX_alpha + CXq_alpha * MAV.c * q/(2 * Va);...
                %     (MAV.C_Y_0 + MAV.C_Y_beta * beta) +( MAV.C_Y_p * MAV.b * p + MAV.C_Y_r * MAV.b *r)/(2 * Va);...
                %     (CZ_alpha) + (CZq_alpha * MAV.c * q)/(2 * Va)];
                %-----------------------------------------------------------------------------------------------------------
                aerodynamic_torque = 0.5 * MAV.rho * Va^2 * MAV.S_wing.*...
                                     [MAV.b * (MAV.C_ell_0 + MAV.C_ell_beta * beta + MAV.C_ell_p * MAV.b * p/(2 * Va)+...
                                      MAV.C_ell_r * MAV.b * r/(2 * Va));...
                                      MAV.c * (MAV.C_m_0 + MAV.C_m_alpha * alpha + MAV.C_m_q * MAV.c * q/(2 * Va));...
                                      MAV.b * (MAV.C_n_0 + MAV.C_n_beta * beta + MAV.C_n_p * MAV.b * p/(2 * Va)+...
                                      MAV.C_n_r * MAV.b * r/(2 * Va))];
            end
            %------------------------------------------------------------------------------------------------------------------

            
            
            % control surface effects on dynamics
            %------------------------------------------------------------------------------------------------------------------
            control_force   = (0.5 * MAV.rho * (Va)^2 * MAV.S_wing).*[CXdeltae_alpha * delta(2);...
                                                                    MAV.C_Y_delta_a * delta(1) + MAV.C_Y_delta_r * delta(4);...
                                                                    CZdeltae_alpha * delta(2)];

            control_torque = 0.5 * MAV.rho * Va^2 * MAV.S_wing.* [MAV.b * (MAV.C_ell_delta_a * delta(1) + MAV.C_ell_delta_r * delta(4));...
                                                                  MAV.c * (MAV.C_m_delta_e * delta(2));...
                                                                  MAV.b * (MAV.C_n_delta_a * delta(1) + MAV.C_n_delta_r * delta(4))];
            %------------------------------------------------------------------------------------------------------------------

            
            
            % propulsion model needs tweaking
            %------------------------------------------------------------------------------------------------------------------
            % V_in         = MAV.V_max * delta(3);
            % omega_a      = MAV.rho * (MAV.D_prop^5)*MAV.C_Q0/(2*pi)^2;
            % omega_a      = 1;
            % omega_b      = MAV.rho * (MAV.D_prop^4) * MAV.C_Q1 * Va/(2 * pi) + MAV.KQ * MAV.K_V / MAV.R_motor;
            % omega_c      = MAV.rho * (MAV.D_prop^3) * MAV.C_Q2 * (Va)^2 - MAV.KQ*V_in / MAV.R_motor + MAV.KQ * MAV.i0;
            % omega_prop   = (-omega_b + sqrt(omega_b^2 - 4 * omega_a * omega_c)) / 2 * omega_a;
            % 
            % thrust_force = (MAV.rho *(MAV.D_prop^4) * (MAV.C_T0)* omega_prop^2)/(4 * pi * pi)+...
            %                (MAV.rho * (MAV.D_prop^3) * MAV.C_T1 * Va * omega_prop)/(2 * pi)+...
            %                MAV.rho * MAV.D_prop * MAV.C_T2 * Va^2;
            % 
            % motor_torque = (MAV.rho * (MAV.D_prop^5) * (MAV.C_Q0) * omega_prop^2)/(4 *pi *pi)+...
            %                (MAV.rho * (MAV.D_prop^4) * MAV.C_Q1 * Va * omega_prop)/(2 * pi)+...
            %                MAV.rho * MAV.D_prop * MAV.C_Q2 * Va^2;
            %------------------------------------------------------------------------------------------------------------------
            
            
            
            % propulsion model
            %------------------------------------------------------------------------------------------------------------------
            thrust_force   = 0.5 * MAV.rho * MAV.S_prop * MAV.C_prop * ((MAV.k_motor * delta(3))^2- Va^2);
            motor_torque   = -MAV.k_T_P * (MAV.k_Omega * delta(3))^2;
            %------------------------------------------------------------------------------------------------------------------
            
            
            
            % total force in body frame
            Force = gravity_force + [thrust_force; 0; 0] + aerodynamic_force + control_force;

            
            
            % total torque in body frame
            Torque = aerodynamic_torque + control_torque + [-motor_torque; 0; 0];
                                                                     
            
            
            % output total force and torque
            out = [Force; Torque];
        end
        %----------------------------
        function self=update_true_state(self)
            eulerA = quat2eul(self.state(7:10)');
            self.true_state.pn = self.state(1);  % pn
            self.true_state.pe = self.state(2);  % pd
            self.true_state.h = -self.state(3);  % h
            self.true_state.phi = eulerA(3); % phi
            self.true_state.theta = eulerA(2); % theta
            self.true_state.psi = eulerA(1); % psi
            self.true_state.p = self.state(11); % p
            self.true_state.q = self.state(12); % q
            self.true_state.r = self.state(13); % r
            self.true_state.Va = self.Va;% Va
            self.true_state.alpha = self.alpha;
            self.true_state.beta = self.beta;
            self.true_state.Vg = sqrt(self.state(4)^2 + self.state(5)^2 + self.state(6)^2);
            self.true_state.wn = self.wind(1);
            self.true_state.we = self.wind(2);
        end
    end
end