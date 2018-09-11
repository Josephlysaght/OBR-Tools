classdef setupCreator < handle
    
    properties
        figH;
        tab;
        tabgp;
        car;
    end
    
    properties (Access = private)
        parametersToImport = {'chassisParameters','tyreCoefficients','powertrainParameters','aeroParameters','kinematicsData'};
    end
    
    methods
        
        function obj = setupCreator()
            import uiextras.jTree.*
            
            obj.figH = figure('Name','Setup Compare',...
                'MenuBar','none',...
                'Position',[100 100 600 600],...
                'Resize','off');
           
          
            obj.tabgp = uitabgroup(obj.figH);
            obj.tab(1).uiTab = uitab(obj.tabgp,'Title','Import Excel Files');
            %obj.tab(2).uiTab = uitab(obj.tabgp,'Title','Confirmation of Setup');
   
            for a=1:size(obj.tab,2)
                obj.tab(a).mainPanel = uipanel('FontSize',12,...
                                        'Position',[0 0.1 1 .9],...
                                        'Parent',obj.tab(a).uiTab);
                obj.tab(a).savePanel = uipanel('FontSize',12,...
                                        'BackgroundColor','white',...
                                        'Position',[0 0 1 .1],...
                                        'Parent',obj.tab(a).uiTab);
                obj.tab(a).saveBtn = uicontrol('Parent',obj.tab(a).savePanel,'String','Save Setup',...
                                        'Position', [455 7.5 100 40],'Style', 'pushbutton',...
                                        'Callback',@obj.saveCarSetup);
                obj.tab(a).exitBtn = uicontrol('Parent',obj.tab(a).savePanel,'String','Exit',...
                                        'Position', [15 7.5 100 40],'Style', 'pushbutton', ...
                                        'Callback',@obj.exit);
            end
            
            for a = 1:size(obj.parametersToImport,2)
                obj.tab(1).importButton(a) = uicontrol('Parent',obj.tab(1).mainPanel,'String',obj.parametersToImport{a},...
                                            'Position', [100 ((a*((600*0.75)/size(obj.parametersToImport,2)))-20) 400 40],'Style', 'pushbutton', ...
                                            'UserData',obj.parametersToImport{a},'Callback',@obj.import);
            end
            
        end
        
        function importCarSetup(obj)
            chassisParameters = obj.car.chassisParameters.path;
            tyreCoefficients = obj.car.tyreCoefficients.path;
            powertrainParameters = obj.car.powertrainParameters.path;
            aeroParameters = obj.car.aeroParameters.path;
            kinematicsData = obj.car.kinematicsData.path;
            obj.car.setup = createCarStruct(chassisParameters,tyreCoefficients,powertrainParameters,aeroParameters,kinematicsData);
        end
        
        function saveCarSetup(obj, varargin)
            if isfield(obj.car,'setup')
                [file, path] = uiputfile('.mat');
                Car = obj.car.setup;
                save([path,file],'Car');
            else
                warndlg('Setup Configuration not Complete, make sure all fields are filled.');
            end
        end
        
        function exit(obj, varargin)
            delete(obj.figH);
        end
        
        function obj = import(obj, varargin)
            if ~isempty(strfind(varargin{1}.String,':'))
                num = strfind(varargin{1}.String,':');
                importName = varargin{1}.String;
                importName = importName(1:num-1);
            else
                importName = varargin{1}.String;
            end
            [file,path] = uigetfile('*.xlsx');
            obj.car.(importName).path = strcat(path,file);
            
            a = find(strcmp(importName,obj.parametersToImport));
            delete(obj.tab(1).importButton(a))
            obj.tab(1).importButton(a) = uicontrol('Parent',obj.tab(1).mainPanel,...
                                    'String',[importName,': ',file],...
                                    'Position', [100 ((a*((600*0.75)/size(obj.parametersToImport,2)))-20) 400 40],...
                                    'Style', 'pushbutton', ...
                                    'UserData',importName,'Callback',@obj.import);
                                
            for a = 1:size(obj.parametersToImport,2)
                if ~isfield(obj.car,(obj.parametersToImport{a}))
                    return;
                end
            end
            
            obj.importCarSetup();
            %obj.updateTable();
        end
        
        function updateTable(obj)
            array = [];
            hold = (struct2cell(obj.car.setup.Aero));
            holdName = (fieldnames(obj.car.setup.Aero));
            array = [hold];
            arrayName = [holdName];
            
            hold = (struct2cell(obj.car.setup.Chassis));
            holdName = (fieldnames(obj.car.setup.Chassis));
            array = [array;hold];
            arrayName = [arrayName;holdName];
            
            hold = (struct2cell(obj.car.setup.Kin));
            holdName = (fieldnames(obj.car.setup.Kin));
            array = [array;hold];
            arrayName = [arrayName;holdName];
            
            hold = (struct2cell(obj.car.setup.Powertrain));
            holdName = (fieldnames(obj.car.setup.Powertrain));
            array = [array;hold];
            arrayName = [arrayName;holdName];
            
            hold = (struct2cell(obj.car.setup.Tyre));
            holdName = (fieldnames(obj.car.setup.Tyre));
            array = [array;hold];
            arrayName = [arrayName;holdName];
            
            obj.tab(2).table = uitable(obj.tab(2).mainPanel,'RowName',arrayName,...
                'Data',array,'ColumnName',{'Name','Value'},'ColumnEditable',[false true]);
        end
        
    end
    
end