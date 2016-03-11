function [NextState, reward,failed] = RunCBStep(Sim,State,ControllerParams)


% Set up the terrain
start_slope = 0;
Sim.Env = Sim.Env.Set('Type','inc','start_slope',start_slope);

% Set up the controller
Sim.Con = Sim.Con.ClearTorques();
Sim.Con = Sim.Con.Set('omega0',1.2666,'P_LegE',0.5973,'FBType',0);
Sim.Con = Sim.Con.AddPulse('joint',1,'amp',-7.3842,'offset',0.1268,'dur',0.07227);
Sim.Con = Sim.Con.AddPulse('joint',2,'amp',5.1913,'offset',0.1665,'dur',0.0537);

% Simulation parameters
%Sim.IC = [0., 0., 0., 0., 0.];
Sim.IC = [0.1393442, -0.1393442, -0.5933174, -0.4680616, 0.8759402];


% Set internal parameters (state dimensions, events, etc)
Sim = Sim.Init();

% Some more simulation initialization
Sim.Mod.LegShift = Sim.Mod.Clearance;
% Sim.Con = Sim.Con.HandleEvent(1, Sim.IC(Sim.ConCo));
Sim.Con = Sim.Con.HandleExtFB(Sim.IC(Sim.ModCo),Sim.IC(Sim.ConCo));

% Simulate
Sim = Sim.Run();


% Calculate eigenvalues
if Sim.Out.Type == 5
    [EigVal,EigVec] = Sim.Poincare();
    % Do some plots
    disp(EigVal);
else
    EigVal = 2*ones(4,1);
    disp(Sim.Out.Text);
end

NextState = 1;
reward = 1;

end