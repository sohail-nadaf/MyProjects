function dryd = dryden(va)
% Dryden Parameters (low altitude, medium turbulence)

dryd.L_u = 200;
dryd.L_v = 200;
dryd.L_w = 50;
dryd.sig_u = 2.12;
dryd.sig_v = 2.12;
dryd.sig_w = 1.4;

% A and C matrices in a controllable canonical form of Dryden Transfer
% Functions 


dryd.U_a = dryd.sig_u*sqrt(2*va/(pi*dryd.L_u)); 
dryd.U_c = 1;
dryd.U_d = va/dryd.L_u;
dryd.Unum = [dryd.U_a];
dryd.Uden = [1 dryd.U_d];

dryd.V_a = dryd.sig_v*sqrt(3*va/(pi*dryd.L_v));
dryd.V_b = dryd.sig_v*sqrt(3*va/(pi*dryd.L_v))*va/(sqrt(3)*dryd.L_v);
dryd.V_c = 2*va/dryd.L_v;
dryd.V_d = va/dryd.L_v;
dryd.Vnum = [dryd.V_a dryd.V_b];
dryd.Vden = [1 dryd.V_c dryd.V_d];


dryd.W_a = dryd.sig_w*sqrt(3*va/(pi*dryd.L_w));
dryd.W_b = dryd.sig_w*sqrt(3*va/(pi*dryd.L_w))*va/(sqrt(3)*dryd.L_w);
dryd.W_c = 2*va/dryd.L_w;
dryd.W_d = va/dryd.L_w;
dryd.Wnum = [dryd.W_a dryd.W_b];
dryd.Wden = [1 dryd.W_c dryd.W_d];
end
