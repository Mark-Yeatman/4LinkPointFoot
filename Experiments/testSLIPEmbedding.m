clear all
path(pathdef)
addpath('Experiments\')
addpath(genpath('Analysis\'))
addpath(genpath('UtilityFunctions\'))
addpath(genpath('Model\'))
addpath('Experiments\Videos\')
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

%Environment Parameters
flowdata.Parameters.Environment.slope = slope; %ground slope
flowdata.Parameters.Environment.g = g;
flowdata.Parameters.dim = 14;        %state variable dimension
 
%Biped Parameters
flowdata.Parameters.Biped.asvec =[Mtor Mth Ms Itorz Ithz Isz ltor lth ls rtor rth rs g];

%Energy Function
flowdata.E_func = @MechE_func;

%Events
flowdata.setPhases({'DSupp','SSupp'})
e1 = struct('name','TrailLift','nextphase','SSupp','nextconfig','');
e2 = struct('name','L2','nextphase','SSupp','nextconfig','');
e3 = struct('name','FootStrike','nextphase','DSupp','nextconfig','');
flowdata.Phases.DSupp.events = {e1,e2};
flowdata.Phases.SSupp.events = {e3};
flowdata.End_Step.event_name = {'FootStrike'};%{'L2','TrailLift'};
flowdata.End_Step.map = @map_End_Step;

flowdata.Impacts.TrailLift.map = @genSwingTraj;
flowdata.Impacts.L2.map = @genSwingTraj;
flowdata.Parameters.Biped.alpha = deg2rad(70);

%Set initial phase and contact conditions
flowdata.State.c_phase = 'DSupp';
flowdata.State.c_configs = {};
flowdata.State.Eref = 0;
flowdata.setImpacts();

%Control
flowdata.Controls.Internal = {@SLIP_Control2};
flowdata.Parameters.SLIP.k = 12250;
flowdata.Parameters.SLIP.L0 = 0.94;
flowdata.Parameters.SLIP.d = 1000;
flowdata.Parameters.SLIP.Md = 70;

flowdata.Parameters.SLIP.tor0 = deg2rad(-5);

flowdata.Parameters.SLIP.kappa = 0.1;
flowdata.Parameters.SLIP.Omega = diag([0,1]);

flowdata.Parameters.SLIP.g_foot =  1.923204258900076e+02;

%Outputs
flowdata.PhaseOutputFuncs = {@Lambda,@SLIPEnergy};
flowdata.WalkOutputFuncs = {@FootSwPos,@FootSwVel,@FootStPos,@FootStVel};

B = eye(7,7);
B = B(:,3:end);
flowdata.Parameters.B = B;
%Load initial condition
load('xi_slip_dsupp.mat')
xi_slip_dsupp(7) = deg2rad(-5);
xi_slip_dsupp(14) = 0;
flowdata.tspan = 0.75;
flowdata.State.Eref = SLIP_E_func(xi_slip_dsupp');
[fstate, xout, tout, out_extra] = walk(xi_slip_dsupp,50);

videopath = 'Experiments\Videos\';
prompt_in = input("Animate?: y/n \n","s");
if strcmp(prompt_in,"y")
    animate(@drawBiped,xout,tout,out_extra,1,strcat(videopath,'testSLIPEmbedding'))
end