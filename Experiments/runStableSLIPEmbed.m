clear all
path(pathdef)
addpath('Experiments\')
addpath(genpath('Analysis\'))
addpath(genpath('UtilityFunctions\'))
addpath(genpath('Model\'))
addpath('Experiments\Videos\')

load StableSLIPEmbedWorkspace2.mat
[fstate, xout, tout, out_extra] = walk(fstate,5);

videopath = 'Experiments\Videos\';
prompt_in = input("Animate?: y/n \n","s");
if strcmp(prompt_in,"y")
    animate(@drawBiped,xout,tout,out_extra,1,strcat(videopath,'testSLIPEmbedding'))
end

prompt_in = input("Plots?: y/n \n","s");
if strcmp(prompt_in,"y")
    basicplots
end

