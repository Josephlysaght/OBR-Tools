classdef setupCompare < handle
    
    properties
        figH;
        setupList;
        plots;
        plotsTab;
        menu;
    end
    
    properties (Access = private)
        setupIcon = which('Icons/gear.png');
        tabsLoc = which('Setup Compare/tabPlots.mat');
        colors = {'red','blue','green','magenta','cyan','yellow','black'};
    end
    
    methods
        
        function obj = setupCompare()
            import uiextras.jTree.*
            
            ans = open(obj.tabsLoc);
            obj.plotsTab = ans.plotsTab;
            
            obj.figH = figure('Name','Setup Compare',...
                'MenuBar','none',...
                'Position',[100 100 1200 600],...
                'Resize','off');
            obj.setupList.panel = uipanel('Parent',obj.figH,'Title','Setup List','FontSize',12,...
             'BackgroundColor','white',...
             'Position',[0 0 .25 1]);
            obj.plots.panel = uipanel('Parent',obj.figH,'Title','Plots','FontSize',12,...
              'Position',[.25 0 .75 1]);
          
            obj.plots.tabgp = uitabgroup(obj.plots.panel);
            for a = 1:size(obj.plotsTab,2)
                obj.plots.plotTab(a).tab = uitab(obj.plots.tabgp,'Title',obj.plotsTab(a).name);
            end
            
            for a = 1:size(obj.plots.plotTab,2)
                obj.plots.plotTab(a).tabAxis(a) = axes('Parent',obj.plots.plotTab(a).tab);
                if mod(size(obj.plotsTab(a).Y,2),2)
                    subPlotX = 1;
                    subPlotY = size(obj.plotsTab(a).Y,2);
                else
                    subPlotX = size(obj.plotsTab(a).Y,2)/2;
                    subPlotY = size(obj.plotsTab(a).Y,2)/2;
                end
                for z = 1:size(obj.plotsTab(a).Y,2)
                    obj.plots.plotTab(a).plot(z) = subplot(subPlotX,subPlotY,z);
                end
            end
            
            obj.setupList.addSetupBtn = uicontrol('Parent',obj.setupList.panel,'Style', 'pushbutton', 'String', 'Add Setup',...
                'Position', [22.5 20 250 20],...
                'Callback', @obj.addSetup);
            
            obj.setupList.treePanel = uipanel('Parent',obj.setupList.panel,...
                'BackgroundColor','white','BorderType','none',...
                'Position',[0 0.1 1 0.9]);
            
            obj.setupList.Tree = Tree('Parent',obj.setupList.treePanel);
            
            obj.menu.contextMenu = uicontextmenu('Parent',obj.figH);        
            obj.menu.deleteBtn = uimenu(obj.menu.contextMenu,'Label','Delete Setup','Callback',@obj.removeSetup);
            
            obj.setupList.number = 0;
        end
        
        function addSetup(obj, varargin)
            import uiextras.jTree.*
            
            if obj.setupList.number ~= 0
                a = size(obj.setupList.Setup,2) + 1;
            else
                a = 1;
            end
            obj.setupList.number = a;
            
            [file,path] = uigetfile('*.mat');
            
            colorNum = a/7;
            integ = floor(colorNum);
            colorNum = (colorNum-integ)*7;
            
            obj.setupList.Setup(a).color = obj.colors{colorNum};
            
            obj.setupList.Setup(a).mainNode = TreeNode('Name',['<html><font color="',obj.setupList.Setup(a).color,'">',file(1:end-4),'</html>'],...
                                                    'Parent', obj.setupList.Tree.Root,'UserData',a);
            %setIcon(obj.setupList.Setup(a).mainNode,obj.setupIcon);
            %set(obj.setupList.Setup(a).mainNode,'Color',obj.setupList.Setup(a).color)
            set(obj.setupList.Setup(a).mainNode,'UIContextMenu',obj.menu.contextMenu)
            
            obj.setupList.Setup(a).variables = load([path,file]);
            
            f = waitbar(0,'Please wait... Running Accel Sim');
            obj.setupList.Setup(a).accelData = LapSim_acceleration(obj.setupList.Setup(a).variables.Car);
            waitbar(.33,f,'Running Skidpad Sim');
            obj.setupList.Setup(a).skidpadData = LapSim_skidpad(obj.setupList.Setup(a).variables.Car);
            waitbar(.67,f,'Running Cornering Sweep');
            obj.setupList.Setup(a).corneringSweepData = LapSim_pureCorneringSweep(obj.setupList.Setup(a).variables.Car);
            waitbar(1,f,'Finishing');
            close(f)
            
            aeroBal80 = obj.setupList.Setup(a).accelData.fAeroBalance(find(min(abs(obj.setupList.Setup(a).accelData.speed-22.222))));
            obj.setupList.Setup(a).aeroBalNode = TreeNode('Name',['Aero Balance: ',num2str(aeroBal80*100),'%'],'Parent',obj.setupList.Setup(a).mainNode,'TooltipString','Aero Balance at 180kph');
            %set(obj.setupList.Setup(a).mainNode,'Color',obj.setupList.Setup(a).color)
            
            obj.setupList.Setup(a).skidpadTimeNode = TreeNode('Name',['Skidpad Time: ',num2str(obj.setupList.Setup(a).skidpadData.time),'s'],'Parent',obj.setupList.Setup(a).mainNode,'TooltipString','Simulated Skipad Time');
            %set(obj.setupList.Setup(a).skidpadTimeNode,'Color',obj.setupList.Setup(a).color)
            obj.setupList.Setup(a).accelTimeNode = TreeNode('Name',['Accel Time: ',num2str(obj.setupList.Setup(a).accelData.time(end)),'s'],'Parent',obj.setupList.Setup(a).mainNode,'TooltipString','Simulated Acceleration Time');
            %set(obj.setupList.Setup(a).accelTimeNode,'Color',obj.setupList.Setup(a).color)
            
            fWheelRate=(obj.setupList.Setup(a).variables.Car.Chassis.fSpring/1000)/obj.setupList.Setup(a).variables.Car.Chassis.fMotionRatio^2;
            rWheelRate=(obj.setupList.Setup(a).variables.Car.Chassis.rSpring/1000)/obj.setupList.Setup(a).variables.Car.Chassis.rMotionRatio^2;
            rollStiffnessFSpring=((obj.setupList.Setup(a).variables.Car.Chassis.fTrack/1000)^2*tand(1)*(fWheelRate*1000))/2;
            rollStiffnessRSpring=((obj.setupList.Setup(a).variables.Car.Chassis.rTrack/1000)^2*tand(1)*(rWheelRate*1000))/2;
            
            fStiffness=obj.setupList.Setup(a).variables.Car.Chassis.fARB+rollStiffnessFSpring;
            rStiffness=obj.setupList.Setup(a).variables.Car.Chassis.rARB+rollStiffnessRSpring;
            
            mechBal=100*(fStiffness/(fStiffness+rStiffness));
            obj.setupList.Setup(a).accelTimeNode = TreeNode('Name',['Mech Balance: ',num2str(mechBal),'%'],'Parent',obj.setupList.Setup(a).mainNode,'TooltipString','Simulated Roll Distribution');
            
            plotSetup(obj,a)
        end
        
        function plotSetup(obj, i)
            for a = 1:size(obj.plots.plotTab,2)
                for b = 1:size(obj.plotsTab(a).X,2)
                    hold(obj.plots.plotTab(a).plot(b), 'on')
                    obj.setupList.Setup(i).tab(a).plot(b) = plot(eval(['obj.setupList.Setup(i).',obj.plotsTab(a).X{b}]),...
                                                        eval(['obj.setupList.Setup(i).',obj.plotsTab(a).Y{b}]),...
                                                        'Parent', obj.plots.plotTab(a).plot(b),...
                                                        'Color',obj.setupList.Setup(i).color);
                    xlabel(obj.plots.plotTab(a).plot(b),obj.plotsTab(a).X{b});
                    ylabel(obj.plots.plotTab(a).plot(b),obj.plotsTab(a).Y{b});
                    hold(obj.plots.plotTab(a).plot(b), 'off')
                end
            end

        end
        
        function removeSetup(obj, varargin)
            a = obj.setupList.Tree.SelectedNodes.UserData;
            
            delete(obj.setupList.Setup(a).mainNode);
            delete(obj.setupList.Setup(a).aeroBalNode);
            delete(obj.setupList.Setup(a).skidpadTimeNode);
            delete(obj.setupList.Setup(a).accelTimeNode);
            
            for y=1:size(obj.setupList.Setup(a).tab,2)
                for z=1:size(obj.setupList.Setup(a).tab(y).plot,2)
                    delete(obj.setupList.Setup(a).tab(y).plot(z));
                end
            end
            
        end
        
    end
    
end
            