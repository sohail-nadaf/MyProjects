classdef spacecraft_viewer < handle
    %--------------------------------
    properties
        body_handle
    	Vertices
    	Faces
    	facecolors
        plot_initialized
    end
    %--------------------------------
    methods
        %----------------------------
        function self = spacecraft_viewer
            self.body_handle = [];
            [self.Vertices, self.Faces, self.facecolors] = self.define_spacecraft();
            self.plot_initialized = 0;           
        end
        %---------------------------
        function self=update(self, state)
            if self.plot_initialized==0
                figure(1); clf;
                self=self.drawBody(state.pn, state.pe, -state.h,...
                                   state.phi, state.theta, state.psi);
                title('Spacecraft')
                xlabel('East')
                ylabel('North')
                zlabel('-Down')
                view(32,47)  % set the vieew angle for figure
                axis([-1000,1000,-1000,1000,-1000,1000]);
                hold on
                grid on
                %axis auto
                self.plot_initialized = 1;
            else
                self=self.drawBody(state.pn, state.pe, -state.h,... 
                                   state.phi, state.theta, state.psi);

            end
        end
        %---------------------------
        function self = drawBody(self, pn, pe, pd, phi, theta, psi)
            Vertices = self.rotate(self.Vertices, phi, theta, psi);   % rotate rigid body  
            Vertices = self.translate(Vertices, pn, pe, pd);     % translate after rotation
            % transform vertices from NED to ENU 
            R = [...
                0, 1, 0;...
                1, 0, 0;...
                0, 0, -1;...
                ];
            Vertices = R*Vertices;
            if isempty(self.body_handle)
                self.body_handle = patch('Vertices', Vertices', 'Faces', self.Faces,...
                                             'FaceVertexCData',self.facecolors,...
                                             'FaceColor','flat');
            else
                set(self.body_handle,'Vertices',Vertices','Faces',self.Faces);
                drawnow
            end
        end 
        %---------------------------
        function pts=rotate(self, pts, phi, theta, psi)
            % define rotation matrix (right handed)
            R_roll = [...
                        1, 0, 0;...
                        0, cos(phi), sin(phi);...
                        0, -sin(phi), cos(phi)];
            R_pitch = [...
                        cos(theta), 0, -sin(theta);...
                        0, 1, 0;...
                        sin(theta), 0, cos(theta)];
            R_yaw = [...
                        cos(psi), sin(psi), 0;...
                        -sin(psi), cos(psi), 0;...
                        0, 0, 1];
            R = R_roll*R_pitch*R_yaw;   % inertial to body
            R = R';  % body to inertial
            % rotate vertices
            pts = R*pts;
        end
        %---------------------------
        % translate vertices by pn, pe, pd
        function pts = translate(self, pts, pn, pe, pd)
            pts = pts + repmat([pn;pe;pd],1,size(pts,2)); %tile the position vector
        end
        %---------------------------
        function [V, F, colors] = define_spacecraft(self)
            % Define the vertices (physical location of vertices)
           V = 25.*[...
                2    0    0;... % point 1
                1   0.3    -0.3;... % point 2
                1   -0.3    -0.3;... % point 3
                1   -0.3    0.3;... % point 4
                1   0.3    0.3;... % point 5
                -4.5   0   0;... % point 6
                0   3   0;... % point 7
                -1.5   3   0;... % point 8
                -1.5  -3  0;... % point 9
                0 -3  0;... % point 10
                -3.5  1.5  0;... % point 11
                -4.5  1.5  0;... % point 12
				-4.5  -1.5  0;... % point 13
				-3.5  -1.5  0;... % point 14
				-3.5  0  0;... % point 15
				-4.5  0  -1.5;... % point 16
            ]';

            % define faces as a list of vertices numbered above
            F = [...
                    1, 2,  3;...  % nose top
                    1, 3,  4;...  % nose left
                    1, 4,  5;...  % nose bottom
					1, 5,  2;...  %nose right
                    2, 3,  6;...  % top fuse
                    3, 4,  6;...  % left fuse
                    4, 5,  6;... % bottom fuse
					5, 2,  6;... % right fuse
					7, 8,  10;... % Wing
					8, 9,  10;... % Wing
                    11, 12, 14;... %Htail
                    12, 13, 14;... % Htail
					15, 6,  16;... % V Tail
                    ];

            % define colors for each face    
            myred = [1, 0, 0];
            mygreen = [0, 1, 0];
            myblue = [0, 0, 1];
            myyellow = [1, 1, 0];
            mycyan = [0, 1, 1];

            colors = [...
                myyellow;... % nose
				myyellow;... % nose
				myyellow;... % nose
				myyellow;... % nose
                myblue;...   % fuse
                myblue;...   % fuse
                myblue;...   % fuse
                myblue;...   % fuse
                mygreen;...  % Wing
                mygreen;...  % Wing
				mygreen;...  % H stab
                mygreen;...  % H stab
				myblue;...   % V Stab
                ];
        end
    end
end