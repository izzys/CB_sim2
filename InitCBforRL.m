function [Sim] = InitCBforRL(Sim)

Sim.EndCond = [1,1]; % Run one step

% Set up the compass biped model
Sim.Mod = Sim.Mod.Set('damp',0,'I',0);

% Set up the terrain
start_slope = 0;
Sim.Env = Sim.Env.Set('Type','inc','start_slope',start_slope);
% leadway = 5;
% parK = 0.01;
% Sim.Env = ...
%     Sim.Env.Set('Type','inf','start_slope',0,'parK',parK,'start_x',leadway);

% Set up the controller
% Sim.W = [-7.3842 0.1268 0.199;
%          5.1913  0.1665  0.22];
%  Sim.W = [0  0  0;
%           0  0  0];
% Sim.W = [-2 0.1 0.2;
%           1  0.3  0.4];
Sim.W =  [ -1.835344765578253   0.191831900555850   0.618291318676476
   0.831617566940014   0.129778297041035   0.461870500988952];
   
   
Sim.Sigma = eye(length(Sim.W(:)))*Sim.sigma;

Sim.const_IC = [0.1393442, -0.1393442, -0.5933174, -0.4680616, 0.8759402];
Sim.random_IC = [0.1393442, -0.1393442, -0.5933174, -0.4680616, 0.8759402];

% Sim.const_IC = [0, 0,0, 0, 0];

%Set state discretization:

% x1 = theta1 (==theta2 at impact)
Sim.x1_min = Sim.const_IC(1)-0.5;
Sim.x1_max = Sim.const_IC(1)+0.5;
Sim.x1_dim = 12;

% x2 = theta1 dot
Sim.x2_min = Sim.const_IC(3)-1.5;
Sim.x2_max =  Sim.const_IC(3)+1.5;
Sim.x2_dim = 4;

% x3 = theta4 dot
Sim.x3_min = Sim.const_IC(4)-1.5;
Sim.x3_max = Sim.const_IC(4)+1.5;
Sim.x3_dim = 4;

% x4 = phi
Sim.x4_min = 0;
Sim.x4_max = 1;
Sim.x4_dim = 10;  
        
Sim.Xdim = [Sim.x1_dim Sim.x2_dim Sim.x3_dim Sim.x4_dim];

Sim.Xbounds = [ Sim.x1_min  Sim.x1_max ;
Sim.x2_min  Sim.x2_max ;
Sim.x3_min  Sim.x3_max ;
Sim.x4_min  Sim.x4_max ]; 

Sim.dX(1) = diff([ Sim.x1_min  Sim.x1_max ])/ Sim.x1_dim ;
Sim.dX(2) = diff([ Sim.x2_min  Sim.x2_max ])/ Sim.x2_dim ;
Sim.dX(3) = diff([ Sim.x3_min  Sim.x3_max ])/ Sim.x3_dim ;
Sim.dX(4) = diff([ Sim.x4_min  Sim.x4_max ])/ Sim.x4_dim ;
    
% Sim.Con = Sim.Con.ClearTorques();
Sim.Con = Sim.Con.Set('omega0',1.2666,'FBType',0);
% Sim.Con = Sim.Con.AddPulse('joint',1,'amp',Sim.W(1),'offset',Sim.W(3),'dur',Sim.W(5));
% Sim.Con = Sim.Con.AddPulse('joint',2,'amp',Sim.W(2),'offset',Sim.W(4),'dur',Sim.W(6));

Sim.Adim = 2;
Sim.Wdim = 3;

% Simulation parameters
Sim.tstep = 0.02;
Sim.tend = 100;
% Set internal parameters (state dimensions, events, etc)
Sim = Sim.Init();


% % Sim.Con = Sim.Con.HandleEvent(1, Sim.IC(Sim.ConCo));
% Sim.Con = Sim.Con.HandleExtFB(Sim.IC(Sim.ModCo),Sim.IC(Sim.ConCo));

end