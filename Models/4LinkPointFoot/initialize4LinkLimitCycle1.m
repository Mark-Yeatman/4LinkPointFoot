global Mt Ms Mh Isz Itz lt ls                 %Mechanics parameters
global flowdata

%path(pathdef)
addpath('SLIPConfig\SimulationFunctions\DynamicsFunctions'); 
addpath('SLIPConfig\SimulationFunctions\ConstraintFunctions'); 
addpath('SLIPConfig\SimulationFunctions\ControlFunctions'); 
addpath('SLIPConfig\SimulationFunctions\ImpactFunctions'); 
addpath('SLIPConfig\SimulationFunctions\EnergyFunctions'); 
addpath('SLIPConfig\UtilityFunctions'); 
addpath('SLIPConfig\AnalysisFunctions'); 
addpath('SLIPConfig\SimulationFunctions\KinematicFunctions')

%Load more parameters
load('MassInertiaGeometryCompass.mat')
load('xi1.mat')

flowdata = flowData;
flowdata.E_func = @MechE2_func;
%ode equation handle and tolerenaces
flowdata.eqnhandle = @dynamics;
flowdata.odeoptions = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);

%Flags
flowdata.Flags.silent = false;
flowdata.Flags.ignore = true;

%simulation parameters
flowdata.Parameters.Environment.slope = -deg2rad(3.7);   %ground slope
flowdata.Parameters.dim = 12;        %state variable dimension

%PBC parameters
flowdata.Parameters.KPBC.sat = inf;
flowdata.Parameters.KPBC.k = 0;
flowdata.Parameters.KPBC.omega = diag([0,0,1,1,1,1]);

%SLIP parameters
flowdata.Parameters.SLIP.k = 12250;
flowdata.Parameters.SLIP.L1 = 0.98;
flowdata.Parameters.SLIP.L2 = 0.98;

%Discrete Mappings 
flowdata.setPhases({'SSupp','DSupp'})
flowdata.setConfigs({'KLockSt','KLockSw'})
impactlist =  {'FootStrike','TrailLift'};
n_phaselist = {'DSupp','SSupp'};
n_configlist = {'keep','keep'};
flowdata.setImpacts(impactlist,n_phaselist,n_configlist);
flowdata.End_Step.event_name = 'FootStrike';

%Set initial phase and contact conditions
flowdata.State.c_phase = 'SSupp';
flowdata.State.c_configs = {'KLockSt','KLockSw'};
flowdata.State.Eref = 0;

%ODE options 
flowdata.odeoptions.Events = flowdata.Phases.SSupp.eventf;




