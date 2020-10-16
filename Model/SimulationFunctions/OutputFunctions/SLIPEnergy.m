function Evec= SLIPEnergy(t,y)
    %SLIPENERGY Summary of this function goes here
    %   Detailed explanation goes here
    Evec = zeros(size(t));
    for j=1:length(t)
        Evec(j) = SLIP_E_func(y(j,:)');
    end   
end

