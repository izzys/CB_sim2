function [ Data ] = Analyze( GA, varargin )
%ANALYZE Analyze the performance of the selected evolved controller
%   Run simulations with the selected controller over a range of slopes
%   until it is unable to walk. Calculate performance parameters such as
%   initial conditions, limit cycles, required torque/motor power,
%   required ZMP, etc.

MaxTries = 10;
base_d = 0.05;

switch nargin 
    case 2
        Generation = GA.Progress;
        ID = varargin{1};
    case 3
        Generation = varargin{1};
        ID = varargin{2};
    otherwise
        Generation = GA.Progress;
        TopIDs = GA.GetTopPop(GA.Fittest(1));
        ID = randsample(TopIDs,1);
end

Filename = ['Gen',int2str(ID),'.mat'];
if exist(Filename,'file') == 2
    In = load(Filename);
    Data = In.Data;
    if Data.Done
        return
    end
    
    start_slope = Data.Slopes(end);
    if length(Data.Slopes)<2
        d_slope = base_d;
    else
        d_slope = sign(Data.Slopes(end))*median(abs(diff(Data.Slopes)));
    end
else
    % Initialize output
    Data.Slopes = []; % Slopes
    Data.IC = []; % Initial conditions
    Data.LCx = {}; % Limit cycle states
    Data.LCt = {}; % Limit cycle time
    Data.LCtorques = {}; % Limit cycle Torques
    Data.MTorques = []; % Max torques
    Data.Power = []; % Actuator power required
    Data.EigV = []; % Eigenvalues
    Data.Period = []; % Period
    Data.LCZMP = []; % Limit cycle ZMP
    Data.MZMP = []; % Max ZMP
    start_slope = 0;
    d_slope = 0;
    
    Data.Done = 0;
    
    % Store genome information
    Data.Gen = GA.Gen;
    Data.Seq = GA.Seqs(ID,:,Generation);    
end

% Run simulation starting from slope 0 and then increasing/decreasing slope
Sim = deepcopy(GA.Sim);
Sim.Graphics = 0;
if Sim.Graphics == 1
    Sim.Fig = figure();
end
Sim.PMFull = 1; % Run poincare map on all 5 coords

Sim = Data.Gen.Decode(Sim, Data.Seq);

while ~Data.Done
    Tries = 0;
    while Tries<MaxTries
        % Run simulation
        Asim = WalkOnSlope(Sim,Data,start_slope,start_slope+d_slope);

        if Asim.Out.Type == 5
            % Simulation converged
            fprintf(' - ');
            cprintf('*green','OK!\n')
            
            Data = SavePerformance(Asim,start_slope+d_slope,Data);

            % Keep going to next slope
            if d_slope == 0
                d_slope = base_d;
            else
                start_slope = start_slope+d_slope;
                d_slope = d_slope/abs(d_slope)*base_d;
            end
            break
        else
            fprintf(' - ');
            cprintf('*red','FAILED!\n')
            disp(Asim.Out.Text);
            
            if Asim.Out.Type ~= 0
                % Try running from 0 to the required slope
                Asim = WalkOnSlope(Sim,Data,0,start_slope+d_slope);

                if Asim.Out.Type == 5
                    % Simulation converged
                    fprintf(' - ');
                    cprintf('*green','OK!\n')

                    Data = SavePerformance(Asim,start_slope+d_slope,Data);

                    % Keep going to next slope
                    if d_slope == 0
                        d_slope = base_d;
                    else
                        start_slope = start_slope+d_slope;
                        d_slope = d_slope/abs(d_slope)*base_d;
                    end
                    break
                else
                    % Simulation failed to converge
                    fprintf(' - ');
                    cprintf('*red','FAILED!\n')
                    disp(Asim.Out.Text);

                    % Try the next slope
                    if d_slope>0
                        d_slope = d_slope+base_d;
                    else
                        d_slope = d_slope-base_d;
                    end
                    Tries = Tries + 1;
                end
            else
                % Try the next slope
                if d_slope>0
                    d_slope = d_slope+base_d;
                else
                    d_slope = d_slope-base_d;
                end
                Tries = Tries + 1;
            end
        end
    end
    
    if Tries == MaxTries
        % Simulation failed to converge, start checking negative slopes
        if d_slope > 0
            start_slope = 0;
            d_slope = -base_d;
        else
            Data.Done = 1;
            break;
        end
    end
end

% Sort data
LastUp = find(diff(Data.Slopes)<0,1,'first');
NSlopes = length(Data.Slopes);
Fields = fieldnames(Data);
for f = 1:length(Fields)
    if strcmp(Fields{f},'Done')
        continue
    end
    
    [r,c] = size(Data.(Fields{f}));
    if c == NSlopes
        Data.(Fields{f}) = [fliplr(Data.(Fields{f})(:,LastUp+1:end)),...
                            Data.(Fields{f})(:,1:LastUp)];
    else
        if r == NSlopes
            Data.(Fields{f}) = [flipud(Data.(Fields{f})(LastUp+1:end,:));...
                                Data.(Fields{f})(1:LastUp,:)];
        else
            disp('Error: wrong number of data points');
        end
    end
end
save(Filename,'Data');


    function sim = WalkOnSlope(sim_in, Data, start_s, end_s)
        sim = deepcopy(sim_in);
        sim  = sim.Init();
        fprintf('Processing data on %.2f degrees slope',end_s);
        
        if start_s == 0
            sim.IClimCyc = 0*sim.IC;
        else
            sim.IClimCyc = Data.IC(1:sim.stDim,Data.Slopes == start_s);
        end
        
        sim = sim.WalkOnSlope(start_s, end_s, ~start_s*5, 75);
    end

    function Data = SavePerformance(sim,Slope,Data)
        Data.Slopes(end+1) = Slope;
        Data.Period(end+1,1) = sim.Period(1);
        % Initial conditions (for each step until a full period)
        Coords = 1:sim.stDim*sim.Period(1);
        Data.IC(Coords,end+1) = ...
            reshape(sim.ICstore(:,1:sim.Period(1)),[],1); 
        
        % Calculate eigenvalues
        if sim.PMFull
            EachP = sim.stDim;
        else
            EachP = length(sim.ModCo);
        end
        EigV = zeros(EachP*sim.Period(1),1);
        for p = 1:sim.Period(1)
            sim.IClimCyc = sim.ICstore(:,p);
            Coords = 1+EachP*(p-1):EachP*p;
            [EigV(Coords),~] = sim.Poincare();
        end
        Data.EigV(1:length(EigV),end+1) = EigV;
        
        % Prepare simulation for single period evaluation
        sim = sim.SetTime(0,0.003,5);
        sim.Mod.xS = 0; sim.Mod.yS = 0;
        sim.Env = sim.Env.Set('Type','inc','start_slope',Slope);
        sim.EndCond = [1,sim.Period(1)]; % Run for one full period
        sim = sim.Init();
        sim.Mod.LegShift = sim.Mod.Clearance;
        sim.IC = Data.IC(1:sim.stDim,end); % sim.IClimCyc;
        sim.Con = sim.Con.Reset(sim.IC(sim.ConCo));
        sim.Con = sim.Con.HandleExtFB(sim.IC(sim.ModCo),...
                sim.IC(sim.ConCo),sim.Env.SurfSlope(sim.Mod.xS));
    
        sim = sim.Run();
        
        Data.LCx{end+1} = sim.Out.X; % Limit cycle states
        Data.LCt{end+1} = sim.Out.T; % Limit cycle time
        Data.Period(end,2) = max(sim.Out.T);
        Data.LCtorques{end+1} = sim.Out.Torques; % Limit cycle torques
        
        % Deal with torques
        % Separate impulses
        [Torques, Impulses] = sim.GetCleanPulses();
        Torques = [Torques, Impulses];
        % Calculate max torque (positive or negative)
        mT = min(Torques,[],1); MT = max(Torques,[],1);
        Moverm = find(abs(MT)>=abs(mT));
        moverM = find(abs(MT)<abs(mT));
        Data.MTorques(end+1,Moverm) = MT(Moverm);
        Data.MTorques(end,moverM) = mT(moverM);
        
        % Actuator power required
        AnkPower = Torques(:,1).*sim.Out.X(:,3);
        HipPower = Torques(:,2).*(sim.Out.X(:,4)-sim.Out.X(:,3));
        Data.Power(end+1,:) = [max(abs(AnkPower)); max(abs(HipPower))];
        
        % ZMP
        NT = length(sim.Out.T);
        % Process the data
        GRF = zeros(NT,2);
        ZMP = zeros(NT,1);
        for t=1:NT
            thisX = sim.Out.X(t,sim.ModCo);
            GRF(t,:) = sim.Mod.GetGRF(thisX)';
            ZMP(t) = Torques(t,1)/GRF(t,2); % Ankle torque/GRFy
        end

        Data.LCZMP{end+1} = ZMP;
        Data.MZMP(end+1,:) = [max(ZMP); min(ZMP)]; % ZMP front; back
        
        % Save progress
        save(Filename,'Data');
    end
end

