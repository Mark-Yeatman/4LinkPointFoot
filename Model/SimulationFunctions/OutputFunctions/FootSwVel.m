function p = FootSwVel(t,y,~)
    %COMPOSOUTPUT Summary of this function goes here
    %   Detailed explanation goes here
    global flowdata
    params = flowdata.Parameters.Biped.asvec;
    p = zeros(length(t),2);
    for i=1:length(t)
       temp =  Foot_Sw_vel_func(y(i,:)',params);
       p(i,:) = temp(1:2,4);
    end
end

