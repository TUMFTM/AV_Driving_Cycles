%% Plot all motor diagrams
%check path
folder = fileparts(which('path_characteristic')); %path where path_characteristic.m is stored
path='02_Characteristics_Diagrams';
path = [folder,'/',path,'/','PSM']; %creates path to characteristic diagrams

%Load characteristic Folder
files = dir([path,'/*.mat']);

%number of motors
num_mot=length(files);
%convert to cell
files=struct2cell(files);

%One Subplot per Torque



    for i=1:(num_mot)
        %Assign Motor name
        folder=files{2,i};
        Name=files{1,i};
        Motor_name=[folder,'\',Name];
        
        clear LDS_Lorenzo
        clear LDS_Diagramm
        
        %Load file
        load(Motor_name)
        
        %Plot
        if exist('LDS_Lorenzo')
            n=flipud(LDS_Lorenzo.n);
            T=flipud(LDS_Lorenzo.M);
        else 
            exist('LDS_Diagramm')
            n=flipud(LDS_Diagramm.n);
            T=flipud(LDS_Diagramm.M);
        end
        plot_mot=subplot(7,7,i);
        if exist('LDS_Lorenzo')
             contourf(plot_mot,n,T,LDS_Lorenzo.eta)
        else exist('LDS_Diagramm')
            contourf(plot_mot,n,T,LDS_Diagramm.eta) 
        end
        title(Name)
        
    end