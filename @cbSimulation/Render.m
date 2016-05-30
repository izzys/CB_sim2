function status = Render(sim,t,X,flag)
% Renders the simulation graphics
    switch flag
        case 'init'
            t = t(1);

            if sim.Once
                % Open new figure
%                 if ishandle(sim.Fig) && strcmp(get(sim.Fig,'type'),'figure')
%                     figure(sim.Fig);
%                     if isempty(findobj(gcf,'Type','uicontrol'))
%                         % Make window larger
%                         set(sim.Fig,'Position', [100 100,...
%                             sim.FigWidth sim.FigHeight]);
%                     end
%                 else
                    sim.Fig = sim.plot_model_handle;
%                     % Make window larger
%                     set(sim.Fig,'Position', [100 100,...
%                         sim.FigWidth sim.FigHeight]);
%                 end


                set(sim.plot_model_handle,'LooseInset',get(sim.plot_model_handle,'TightInset')*2)
                cla % clear previous render

                % Initialize COM tranform for "follow" mode
                sim.plotObj.tCOM = hgtransform('Parent',sim.plot_model_handle);

                % Initialize display timer
                sim.plotObj.hTime = uicontrol('Style', 'text',...
                    'String', sprintf(sim.TimeStr,t,X(sim.ConCo),...
                        sim.Env.SurfSlope(sim.Mod.xS)*180/pi,sim.Mod.curSpeed),...
                    'HorizontalAlignment','left',...
                    'FontSize',12,...
                    'Units','normalized',...
                    'Position', [0.91 0.76 0.08 0.12],...
                    'backgroundcolor',get(sim.plot_model_handle,'color')); 
                
                % Initialize convergence display
                sim.plotObj.hConv = uicontrol('Style', 'text',...
                    'String', sprintf(sim.ConvStr,1,'-'),...
                    'HorizontalAlignment','left',...
                    'FontSize',12,...
                    'Units','normalized',...
                    'Position', [0.91 0.7 0.08 0.06],...
                    'backgroundcolor',get(sim.plot_model_handle,'color')); 

                % Add a 'Stop simulation' button
                uicontrol('Style', 'pushbutton', 'String', 'Graphics',...
                    'Units','normalized','FontSize',12,...
                    'Position', [0.92 0.9 0.07 0.05],...
                    'Callback', @sim.StopButtonCb);
                sim.Once = 0;

                % Render torque plots
                if sim.Con.nPulses>0
                    sim.plotObj.hTorques = zeros(sim.nOuts,1);                
                    for to = 1:sim.nOuts
                        sim.plotObj.hTorques(to) = line(sim.Ttime,...
                            sim.Tbase+sim.Thold(to,:)*sim.Tscale,...
                            'parent',sim.plotObj.tCOM,'Color',sim.Colors{to},...
                            'LineWidth',2);
                    end
                end
            end
    end
    
    if ishandle(sim.plotObj.tCOM)==0
        sim.Once = 1;
        status = Render(sim,t,X,flag);
        return
    end
    
    if ~isempty(X)
        % Check time elapsed
        sim.TimeTic = sim.TimeTic+1;
        if sim.TimeTic<sim.RSkip
            status = sim.StopSim;
            return
        else
            sim.TimeTic = 0;
        end
        
        if sim.Follow
            [COMx,~]=sim.Mod.GetPos(X(sim.ModCo),'COM');
            [COMy,~]=sim.Env.Surf(COMx);
            sim.FlMin=COMx-1.25*sim.AR*sim.Mod.L;
            sim.FlMax=COMx+1.25*sim.AR*sim.Mod.L;
            HeightMin=COMy-1/sim.AR*sim.Mod.L;
            HeightMax=COMy+4/sim.AR*sim.Mod.L;

            TCOMx = makehgtform('translate',[COMx-sim.COMx0 COMy-sim.COMy0 0]);
            set(sim.plotObj.tCOM,'Matrix',TCOMx);

            axis( sim.plot_model_handle, [sim.FlMin sim.FlMax HeightMin HeightMax]);
        end

        % Update model render
        sim.Mod = sim.Mod.Render(X(sim.ModCo));
        % Update environment render
        sim.Env = sim.Env.Render(sim.FlMin,sim.FlMax);
        % Update time display
        set(sim.plotObj.hTime,'string',...
            sprintf(sim.TimeStr,t(1),X(sim.ConCo),...
                sim.Env.SurfSlope(sim.Mod.xS)*180/pi,sim.Mod.curSpeed));
        % Update convergence display
        Period = find(sim.stepsSS>0,1,'first');
        if ~isempty(Period)
%             diff = norm(sim.ICstore(:,1) - sim.ICstore(:,1+Period));
            diff = min(sim.ICdiff);
            set(sim.plotObj.hConv,'string',...
                sprintf(sim.ConvStr,diff,int2str(Period)),...
                    'backgroundcolor',[0.5 1 0.5]);
        else
%             diff = norm(sim.ICstore(:,1) - sim.ICstore(:,2));
            diff = min(sim.ICdiff);
            Period = find(sim.ICdiff == diff, 1, 'first');
            set(sim.plotObj.hConv,'string',...
                sprintf(sim.ConvStr,diff,['(',int2str(Period),')']),...
                    'backgroundcolor',get(sim.plot_model_handle,'color'));
        end
        % Update torque display
        if sim.Con.nPulses>0
            sim.Thold(:,1:end-1) = sim.Thold(:,2:end);
            sim.Thold(:,end) = sim.Mod.Torques;
            for to = 1:sim.nOuts
                set(sim.plotObj.hTorques(to),...
                    'XData',sim.Ttime,...
                    'YData',sim.Tbase + sim.Thold(to,:)*sim.Tscale);
            end
        end
    end
    status = sim.StopSim;
    drawnow
end