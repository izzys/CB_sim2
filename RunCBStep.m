function [X, reward,failed] = RunCBStep(Sim,IC,Slope)

    if length(IC) == 4
        gamm = Sim.Env.incline;
        ic_temp = [IC(1)+gamm;-IC(1)+gamm;IC(2);IC(3);IC(4)];
        IC = ic_temp;
    end

    Sim.IC = IC;

    if Sim.first_step
        Sim = Sim.SetTime(0,Sim.tstep,Sim.tend);
        Sim.first_step = 0;
    else
        Sim = Sim.SetTime(Sim.Out.T(end),Sim.tstep,Sim.tend);
    end
    
    % Set internal parameters (state dimensions, events, etc)
%     Sim = Sim.Init();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize simulation properties
    % Set states
%     Sim.stDim = Sim.Mod.stDim + Sim.Con.stDim; % state dimension
%     Sim.ModCo = 1:Sim.Mod.stDim; % Model coord. indices
%     Sim.ConCo = Sim.Mod.stDim+1:Sim.stDim; % Contr. coord. indices
   
%     Sim.nOuts = length(Sim.Con.NeurOutput());
    Sim.nOuts = length(Sim.Con.GetTorques(Sim.W,0));

    % Set events
    Sim.nEvents = Sim.Mod.nEvents + Sim.Con.nEvents;
    Sim.ModEv = 1:Sim.Mod.nEvents; % Model events indices
    Sim.ConEv = Sim.Mod.nEvents+1:Sim.nEvents; % Contr. events indices
    
    Sim.StopSim = 0;
        
    % Set render params
    if Sim.Graphics == 1 && Sim.Once
        if Sim.Fig == 0
            Sim.Once = 1;
        end
        
        % Init window size params
        scrsz = get(0, 'ScreenSize');
        if scrsz(3)>2*scrsz(4) % isunix()
            % If 2 screens are used in Linux
            scrsz(3) = scrsz(3)/2;
        end
        Sim.FigWidth = scrsz(3)-250;
        Sim.FigHeight = scrsz(4)-250;
        Sim.AR = Sim.FigWidth/Sim.FigHeight;
        if isempty(Sim.IC)
            [Sim.COMx0,Sim.COMy0] = Sim.Mod.GetPos(zeros(1,Sim.Mod.stDim),'COM');
        else
            [Sim.COMx0,Sim.COMy0] = Sim.Mod.GetPos(Sim.IC(Sim.ModCo),'COM');
        end
        
        % Init world size params
        Sim.FlMin = Sim.COMx0-1.25*Sim.AR*Sim.Mod.L;
        Sim.FlMax = Sim.COMx0+1.25*Sim.AR*Sim.Mod.L;
        Sim.HeightMin = Sim.COMy0-1/Sim.AR*Sim.Mod.L;
        Sim.HeightMax = Sim.COMy0+4/Sim.AR*Sim.Mod.L;

        % Init torque display params
        if Sim.Con.nPulses>0
            % Set number of steps so a whole cycle of the oscillator
            % will be included
            Sim.nTsteps = ceil(Sim.Con.GetPeriod()/Sim.tstep);
            Sim.Ttime = linspace(Sim.FlMax*0.8,Sim.FlMax*0.95,Sim.nTsteps);
            Sim.Thold = zeros(Sim.nOuts,Sim.nTsteps);
            Sim.Tbase = (Sim.HeightMax+Sim.HeightMin)/2;
            Sim.Tscale = 0.1*(Sim.HeightMax-Sim.HeightMin)/max(abs(Sim.Con.Amp0));
        end
        
        Sim.Mod.curSpeed = 'Computing...';
    end
    
    Sim.StepsTaken = 0;
    Sim.Steps2Slope = [];
    Sim.MinSlope = 0;
    Sim.MaxSlope = 0;
    Sim.ICstore = zeros(Sim.stDim, Sim.nICsStored);
    Sim.ICdiff = ones(1,Sim.nICsStored-1);
    Sim.stepsSS = zeros(1,Sim.nICsStored-1);
    
    % Init Sim.End result
    Sim.Out.Type = 0;
    Sim.Out.Text = 'Reached end of tspan';
    
    % Adapt CPG (if adaptive)
    Sim.Con = Sim.Con.Adaptation(Sim.Env.SurfSlope(Sim.Mod.xS));
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%             
              
    % Some more simulation initialization
    Sim.Mod.LegShift = Sim.Mod.Clearance;

    % set controller values:
%     Sim.Con = Sim.Con.ClearTorques();
%     Sim.Con = Sim.Con.Set('omega0',1.2666,'FBType',0);
%     Sim.Con = Sim.Con.AddPulse('joint',1,'amp',Sim.Wp(1),'offset',Sim.Wp(3),'dur',Sim.Wp(5));
%     Sim.Con = Sim.Con.AddPulse('joint',2,'amp',Sim.Wp(2),'offset',Sim.Wp(4),'dur',Sim.Wp(6));


    % Sim.Con = Sim.Con.HandleEvent(1, Sim.IC(Sim.ConCo));
    Sim.Con.lastPhi = Slope;
    Sim.Con = Sim.Con.HandleExtFB(Sim.IC(Sim.ModCo),Sim.IC(Sim.ConCo),Slope);
    
    Sim.Con = Sim.Con.Reset(Sim.IC(Sim.ConCo));
    
    % Simulate
    Sim = Sim.Run();

    failed = 0;
    reward = 1;
    x = Sim.ICstore(:,1);
    
    if Sim.Out.Type ~= 4
        failed = 1;
        reward = 0;
        x = nan(1,5);
    end
    
  %  State=Sim.Out;
    

    X = [x(1);x(2);x(3);x(4);x(5)];
   
end