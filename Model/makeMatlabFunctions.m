function makeMatlabFunctions()
% Turns the mathematica derivation of the robot dynamics into optimized executable matlab functions. 

    %Where to put the matlab functions
    mylocation = which(mfilename);
    [thisdir,~,~] = fileparts(mylocation);
    idcs   = strfind(thisdir,'\');
    configFolder = thisdir;%(1:idcs(end)-1);

    %Where to get the "mathematica" functions
    SymFuncsPath = strcat(pwd,"\Derivations\ToMatlabOutputs");
    addpath(genpath(SymFuncsPath))
  
    lt = 0;
    ls = 0;
    syms Mtor Mth Ms Itorz Ithz Isz ltor lth ls rtor rth rs g 
    params = [Mtor Mth Ms Itorz Ithz Isz ltor lth ls rtor rth rs g ];
    x = sym('x',[14,1]);
    a = sym('a',[7,1]);

    M_matrix
    C_matrix
    G_matrix

    folder = strcat(configFolder,'\SimulationFunctions\DynamicsFunctions\');
    status = mkdir(folder);
    matlabFunction(M,   'File',strcat(folder,'M_func'),'Vars',{x,params})
    matlabFunction(Cmat,'File',strcat(folder,'C_func'),'Vars',{x,params})
    matlabFunction(G,   'File',strcat(folder,'G_func'),'Vars',{x,params})

    Kinetic_energy
    Potential_energy

    folder = strcat(configFolder,'\SimulationFunctions\EnergyFunctions\');
    status = mkdir(folder);
    matlabFunction(Ke,'File',strcat(folder,'KE_func'),'Vars',{x,params})
    matlabFunction(Pe,'File',strcat(folder,'PE_func'),'Vars',{x,params})
    
    Foot_st_pos
    Knee_st_pos
    Hip_pos
    Knee_sw_pos
    Foot_sw_pos
    Head_pos
    COM_pos
    
    folder = strcat(configFolder,'\SimulationFunctions\KinematicFunctions\');
    status = mkdir(folder);
    matlabFunction(posFst,  'File',strcat(folder,'Foot_St_pos_func'),'Vars',{x,params})
    matlabFunction(posKst,  'File',strcat(folder,'Knee_St_pos_func'),'Vars',{x,params})
    matlabFunction(posH,    'File',strcat(folder,'Hip_pos_func'),'Vars',{x,params})
    matlabFunction(posFsw,  'File',strcat(folder,'Foot_Sw_pos_func'),'Vars',{x,params})
    matlabFunction(posKsw,  'File',strcat(folder,'Knee_Sw_pos_func'),'Vars',{x,params})
    matlabFunction(posHead,  'File',strcat(folder,'Head_pos_func'),'Vars',{x,params})
    matlabFunction(posCoM,  'File',strcat(folder,'COM_pos_func'),'Vars',{x,params})
    
    Foot_st_vel
    Knee_st_vel
    Hip_vel
    Knee_sw_vel
    Foot_sw_vel
    COM_vel
    
    matlabFunction(velFst,  'File',strcat(folder,'Foot_St_vel_func'),'Vars',{x,params})
    matlabFunction(velKst,  'File',strcat(folder,'Knee_St_vel_func'),'Vars',{x,params})
    matlabFunction(velH,    'File',strcat(folder,'Hip_vel_func'),'Vars',{x,params})
    matlabFunction(velFsw,  'File',strcat(folder,'Foot_Sw_vel_func'),'Vars',{x,params})
    matlabFunction(velKsw,  'File',strcat(folder,'Knee_Sw_vel_func'),'Vars',{x,params})
    matlabFunction(dCoM,'File',strcat(folder,'COM_vel_func'),'Vars',{x,params})
    
    Foot_st_Jacobian
    Knee_st_Jacobian
    Hip_Jacobian
    Knee_sw_Jacobian
    Foot_sw_Jacobian

    matlabFunction(sym(js0),  'File',strcat(folder,'Foot_St_Jacobian_func'),'Vars',{x,params})
    matlabFunction(sym(js1),  'File',strcat(folder,'Knee_St_Jacobian_func'),'Vars',{x,params})
    matlabFunction(sym(js2),    'File',strcat(folder,'Hip_Jacobian_func'),'Vars',{x,params})
    matlabFunction(sym(js3),  'File',strcat(folder,'Knee_Sw_Jacobian_func'),'Vars',{x,params})
    matlabFunction(sym(js4),  'File',strcat(folder,'Foot_Sw_Jacobian_func'),'Vars',{x,params})

    Foot_st_Jacobian_Dot
    Knee_st_Jacobian_Dot
    Hip_Jacobian_Dot
    Knee_sw_Jacobian_Dot
    Foot_sw_Jacobian_Dot

    matlabFunction(sym(js0dot),  'File',strcat(folder,'Foot_St_Jacobian_Dot_func'),'Vars',{x,params})
    matlabFunction(sym(js1dot),  'File',strcat(folder,'Knee_St_Jacobian_Dot_func'),'Vars',{x,params})
    matlabFunction(sym(js2dot),    'File',strcat(folder,'Hip_Jacobian_Dot_func'),'Vars',{x,params})
    matlabFunction(sym(js3dot),  'File',strcat(folder,'Knee_Sw_Jacobian_Dot_func'),'Vars',{x,params})
    matlabFunction(sym(js4dot),  'File',strcat(folder,'Foot_Sw_Jacobian_Dot_func'),'Vars',{x,params})

    
    COM_accel
    matlabFunction(ddCoM,'File',strcat(folder,'COM_accel_func'),'Vars',{x,a,params})
    
end
