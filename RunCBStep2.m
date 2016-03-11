function [X, reward,failed] = RunCBStep2(Sim,IC,Slope,foot_ext)
    failed = 0;
    if length(IC) == 4
        gamm = Sim.Env.incline;
        ic_temp = [IC(1)+gamm;-IC(1)+gamm;IC(2);IC(3);IC(4)];
        IC = ic_temp;
    end


    Sim.IC = IC;

    % Set internal parameters (state dimensions, events, etc)
    Sim = Sim.Init();

    % Some more simulation initialization
    if ~foot_ext
        Sim.Mod.LegShift = Sim.Mod.Clearance;
    end
    % Sim.Con = Sim.Con.HandleEvent(1, Sim.IC(Sim.ConCo));
    Sim.Con.lastPhi = Slope;
    Sim.Con = Sim.Con.HandleExtFB(Sim.IC(Sim.ModCo),Sim.IC(Sim.ConCo),Slope);
    
    Sim.Con = Sim.Con.Reset(Sim.IC(Sim.ConCo));
    
    % Simulate
    Sim = Sim.Run();
    if Sim.Out.Type ~= 4
        failed = 1;
    end
    State=Sim.Out;
    x = Sim.ICstore(:,1);
%   X = [(x(1)-x(2))/2;x(3);x(4);x(5)];
    X = [x(1);x(2);x(3);x(4);x(5)];
    reward = 1;
end