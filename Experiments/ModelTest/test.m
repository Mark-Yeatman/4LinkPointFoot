clear all
path(pathdef)
addpath('Experiments\ModelTest\')
addpath(genpath('Analysis\'))
addpath('UtilityFunctions\')
addpath(genpath('Model'))

global flowdata
flowdata = flowData;

%ODE equation handle and tolerances
flowdata.eqnhandle = @dynamics;
flowdata.odeoptions = odeset('RelTol', 1e-10, 'AbsTol', 1e-10);

%Flags
flowdata.Flags.silent = false;
flowdata.Flags.ignore = true;
flowdata.Flags.do_validation = false;

%Parameters
g = 9.81;   %m/(kg s^2)
slope = deg2rad(0); 
Mtor = 40;      %kg 
Mth = 10;        %kg 
Ms = 5;         %kg 
Itorz = 0;     %kg m^2
Ithz = 0;       %kg m^2
Isz = 0;        %kg m^2
ltor = 0.5;    %m
lth = 0.5;      %m
ls = 0.5;       %m
rtor = 0.25/2;  %m
rth = 0.5/2;    %m
rs = 0.5/2;     %m

flowdata.Parameters.Environment.slope = slope; 
flowdata.Parameters.Environment.g = g;
flowdata.Parameters.dim = 14;        
 
flowdata.Parameters.Biped.asvec =[Mtor Mth Ms Itorz Ithz Isz ltor lth ls rtor rth rs g];

%Energy Function
flowdata.E_func = @MechE_func;

%Control
flowdata.Controls.Internal = {};
flowdata.Controls.External = {};

%Discrete Mappings 
flowdata.setPhases({'DSupp'})
flowdata.setConfigs({})

flowdata.Phases.DSupp.events = {};

flowdata.End_Step.event_name = 'FootStrike';
flowdata.End_Step.map = @map_End_Step;

%Outputs
flowdata.PhaseOutputFuncs = {};
flowdata.StepOutputFuncs = {};
flowdata.WalkOutputFuncs = {};

%Set initial phase and contact conditions
flowdata.State.c_phase = 'DSupp';
flowdata.State.c_configs = {};
flowdata.setImpacts();

xi = zeros(1,14);
xi(1:7) = [0,1.5,pi/4,-pi/4,-pi/4,0,0];
flowdata.tspan = 1;
[fstate, xout, tout, out_extra] = walk(xi,1);

videopath = 'Experiments\Videos\';
prompt_in = input("Animate?: y/n \n","s");
if strcmp(prompt_in,"y")
    animate(@drawBiped,xout,tout,out_extra,1,strcat(videopath,'test'))
end