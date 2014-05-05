classdef Controller
    % Version 0.4 - 28/04/2014
    
    % 4 Leaky Integrate and Fire oscillators.
    % A pair of flexor and extensor LIF oscillator for each leg.
    
    properties
        % LIF parameters
        P_reset = 0;
        P_th = 1;
        
        P_LegE = 0.65; % timed leg extension

        % Oscillator's frequency
        omega = 1.106512566;
        omega0 = 1.106512566;
        
        % Initial phase for oscillator:
        P_0 = 0;
                
        stDim = 1; % state dimension
        nEvents = 2; % num. of simulation events
        
        % Controller Output
        nPulses = 0; % Overall number of pulses
        OutM;        % Output matrix (multiplies Switch vector)
        
        Amp = 0;     % Pulse amplitude in N
        Amp0 = 0;    % Base pulse amplitude in N
        Offset = 0;  % Defines beginning of pulse as % of neuron period
        Duration = 0;% Pulse duration as % of neuron period
        
        Switch;      % 0 when off, Amp when pulse is active
        pSoff = 0;   % Phase at which to turn off external inputs
        
        % Feedback
        FBType = 2;  % 0 - no feedback
                     % 1 - single gain for omega and each joint
                     % 2 - individual gains for each pulse
                     
        % Phase reset
        ExtP_reset = []; % set to a certain phase to use phase reset
                         % on foot contact
                         
        % Angular velocity impulses
        FBImpulse = 0; % 0 - no feedback
                       % 1 - add a constant value
                       % 2 - set ang. vel. to certain value
        AngVelImp = [];
        
        % Gains
        kOmega_u = 0.4579;
        kOmega_d = 0.9;
        kTorques_u = 0;
        kTorques_d = 0;
    end
    
    methods
        % %%%%%% % Class constructor % %%%%%% %
        function [NC] = Controller(varargin)
            % Set torque parameters
            NC.nPulses = 4;
            NC.Amp0=[-13.6255 -4.6281 13.6255 0];
            NC.Amp=NC.Amp0;
            NC.OutM = [1 1 0 0; 0 0 1 1];
            
            NC.Offset=[0 0 0 0];
            
            NC.Duration=[0.1 0.4 0.1 0];
            
            NC.Switch=zeros(NC.nPulses,1);
            NC.pSoff=zeros(1,NC.nPulses);
            
            % Set adaptation parameters
            NC.kTorques_u=[95 -443 95 0];
            NC.kTorques_d=[80 -295 80 0];
            
            NC.nEvents=1+NC.nPulses*2+1;
        end
        
        function [NC] = ClearTorques(NC)
            NC.nPulses = 0;
            NC.OutM = 0;
            NC.Amp0 = [];
            NC.Amp = [];
            NC.Offset = [];
            NC.Duration = [];
            NC.Switch = [0,0]';
            NC.pSoff = [];
            if NC.FBType == 2
                NC.kTorques_u = [];
                NC.kTorques_d = [];
            end
            NC.nEvents = 2;
        end
        
        function [Torques] = NeurOutput(NC)
            Torques = NC.OutM*NC.Switch;
        end

        function [per] = GetPeriod(NC)
            per = (NC.P_th-NC.P_reset)/NC.omega;
        end
        
        function [NC] = Adaptation(NC, X)
            if NC.FBType == 0
                % NO FEEDBACK
                NC.omega = NC.omega0;
                NC.Amp = NC.Amp0;
            else
                Phi = (X(1)+X(2))/2;
                
                NC.omega = NC.omega0 + ...
                    min(0,Phi)*NC.kOmega_d + ...    % Phi<0
                    max(0,Phi)*NC.kOmega_u;         % Phi>0
                NC.Amp = NC.Amp0 + ...
                    min(0,Phi)*NC.kTorques_d + ...  % Phi<0
                    max(0,Phi)*NC.kTorques_u;       % Phi>0
            end
        end
        
        % %%%%%% % Derivative % %%%%%% %
        function [Xdot] = Derivative(NC, t, X) %#ok<INUSD,MANUSL>
            Xdot = NC.omega;
        end
        
        % %%%%%% % Events % %%%%%% %
        function [value, isterminal, direction] = Events(NC, X)
            value = ones(NC.nEvents,1);
            isterminal = ones(NC.nEvents,1);
            direction = -ones(NC.nEvents,1);
            
            % Check for firing neuron
            value(1) = NC.P_th - X;

            % Check for leg extension signal (for clearance)
            value(2) = NC.P_LegE - X;
            
            % Check for switching on signal
            value(3:2+NC.nPulses) = NC.Offset/NC.omega - X;
            value(3+NC.nPulses:2+2*NC.nPulses) = ...
                (NC.Offset+NC.Duration)/NC.omega - X;
        end
        
        function [NC,Xa] = HandleEvent(NC, EvID, Xb)
            Xa = Xb;
            switch EvID
                case 1
                    % Neuron fired
                    for i=1:NC.nPulses
                        if NC.Offset(i)==0;
                            % Turn on signal now
                            NC.Switch(i) = NC.Amp(i);
                        end
                    end
                    Xa = NC.P_reset; % reset phase
                case 2
                    % Extend the leg
                    % This is done from the simulation file
                case num2cell(3:2+NC.nPulses)
                    % Switch on signal
                    NC.Switch(EvID-2) = NC.Amp(EvID-2);
                case num2cell(3+NC.nPulses:2+2*NC.nPulses)
                    % Switch off signal
                    NC.Switch(EvID-(2+NC.nPulses)) = 0;
            end
        end
        
        function [NC, Xmod, Xcon] = HandleExtFB(NC, Xmod, Xcon)
            % This function is called when the leg hits the ground
            if NC.FBType > 0
                % Perform adaptation based on terrain slope
                NC = NC.Adaptation(Xmod);
            end
            
            if ~isempty(NC.ExtP_reset)
                % Perform a phase reset
                Xcon = NC.ExtP_reset;
                
                % Check if any event happens at ExtP_reset
                [value, it, dir] = NC.Events(Xcon); %#ok<NASGU,ASGLU>
                EvIDs = find(value == 0);
                for ev = 1:EvIDs
                    [NC,Xcon] = NC.HandleEvent(EvIDs(ev),Xcon);
                end
            end
            
            switch NC.FBImpulse
                case 1 % 1 - add a constant value
                    Xmod(3:4) = Xmod(3:4) + NC.AngVelImp;
                case 2 % 2 - set ang. vel. to certain value
                    Xmod(3:4) = NC.AngVelImp;
            end
        end
        
        function [NC] = LoadParameters(NC,ControlParams)
            % Set CPG Parameters according to ControlParams input
            % ControlParams should include:
            % [ Omega,  leg extend phase, 
            %   Torque 1 strength, offset, duration,
            %   Torque 2 strength, offset, duration,
            %   ...
            %   Torque n strength, offset, duration]
            
            NC.NumTorques=(length(ControlParams)-2)/3;
            NC.NumEvents=1+NC.NumTorques*2+1;

            NC.omega0=ControlParams(1);
            NC.kOmega_up=0;
            NC.kOmega_down=0;

            NC.P_LegE=ControlParams(2);
            
            NC.NSwitch=zeros(1,NC.NumTorques);
            
            NC.NTorque0=zeros(1,NC.NumTorques);
            NC.NOffset=NC.NTorque0;
            NC.NDuration=NC.NTorque0;
            NC.kTorques_up=NC.NTorque0;
            NC.kTorques_down=NC.NTorque0;
            for i=1:NC.NumTorques
                NC.NTorque0(i)=ControlParams(3*i);
                NC.NTorque(i)=NC.NTorque0(i);
                NC.NOffset(i)=ControlParams(3*i+1);
                NC.NDuration(i)=ControlParams(3*i+2);
                NC.kTorques_up(i)=0;
                NC.kTorques_down(i)=0;
            end
        end
        
        function PlotTorques(NC)
            NumSteps=1000;
            TorqueSig=zeros(NC.NumTorques,NumSteps);

            Phase=linspace(0,1,NumSteps);
            for p=1:NumSteps
                for t=1:NC.NumTorques
                    if Phase(p)>=NC.NOffset(t) && Phase(p)<NC.NOffset(t)+NC.NDuration(t)
                        TorqueSig(t,p)=NC.NTorque(t);
                    end
                end
            end        

            plot(Phase,TorqueSig);
        end
        
        function [Time,TorqueSig]=GetTorqueSig(NC,tstep)
            Time=0:tstep:1/NC.omega0+tstep;
            NumSteps=length(Time);
            
            Phase=linspace(0,1,NumSteps);
            TorqueSig=zeros(NC.NumTorques,NumSteps);
            
            for p=1:NumSteps
                for t=1:NC.NumTorques
                    if Phase(p)>=NC.NOffset(t) && Phase(p)<NC.NOffset(t)+NC.NDuration(t)
                        TorqueSig(t,p)=NC.NTorque(t);
                    end
                end
            end        
        end
    
    end
end