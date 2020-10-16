function [mapeigs] = eigenmap(xstart,period,act_period,vector)
% EIGENMAP2   Find the eigenvalues of the system's Poincare return map
%
% Runs a numerical analysis on the initial conditions for a fixed point.
% The limit cycle code calls step2.m, which calls the function eqns3.dll.
%
% Last modified on 01/23/10 by Robert D. Gregg - rgregg@uiuc.edu
%

global flowdata
dim = flowdata.Parameters.dim;
    if nargin == 1
        period = 1;
        act_period = 1;
    elseif nargin == 2
        act_period = period;
    end


    delta = 1E-2;
    deltadot = 1E-2;

    Omega = zeros(dim,length(vector(vector==1)));
    Y = zeros(dim,length(vector(vector==1)));

    if nargin < 2
        period = 1;
    end

    if nargin == 0
        fprintf('Error: must input initial state!\n')
    end

    j=1;
    for i=1:dim
        if vector(i) == 1
            if(i<=dim/2)
                x = xstart; 
                x(i) = x(i) - delta;
                Y(i,j) = delta;
            else
                x = xstart; 
                x(i) = x(i) - deltadot;
                Y(i,j) = deltadot;
            end
            load StableSLIPEmbedWorkspace2.mat flowdata
            disp([i,delta]')
            flowdata.Flags.silent = true;
            [v, xout, tout, out_extra] = walk(x,period); %FUNCTION CALL IS HERE
            Omega(:,j) = v'-xstart';
            Omega(1,j) = 0;
            j=j+1;
        end
    end
    A = Omega/Y;
    mapeigs = eig(A);
    fprintf('max(abs(eigenvalues))=%f\n',max(abs(mapeigs)));
end