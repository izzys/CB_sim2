function [Sim] = InitCBforRL(Sim)

Sim.Graphics = 1;
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
% Sim.Con = Sim.Con.ClearTorques();
% Sim.Con = Sim.Con.Set('omega0',1.2666,'P_LegE',0.5973,'FBType',0);
% Sim.Con = Sim.Con.AddPulse('joint',1,'amp',-7.3842,'offset',0.1268,'dur',0.07227);
% Sim.Con = Sim.Con.AddPulse('joint',2,'amp',5.1913,'offset',0.1665,'dur',0.0537);

% Simulation parameters
Sim = Sim.SetTime(0,0.05,60);


% params for RL:
Sim.Xdim = 5;
Sim.Adim = 2;
Sim.Wdim = 3;



 %Sim.IC = [0., 0., 0., 0., 0.];
%  Sim.IC = [0.1393442, -0.1393442, -0.5933174, -0.4680616, 0.8759402];


% Set internal parameters (state dimensions, events, etc)
Sim = Sim.Init();

% Some more simulation initialization
% Sim.Mod.LegShift = Sim.Mod.Clearance;
% % Sim.Con = Sim.Con.HandleEvent(1, Sim.IC(Sim.ConCo));
% Sim.Con = Sim.Con.HandleExtFB(Sim.IC(Sim.ModCo),Sim.IC(Sim.ConCo));

end