function Tf = Tf(pm)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明

R = pm(1:3,1:3);
p = pm(1:3,4); 

Tf = [   
    R        zeros(3,3)  
    C3(p)*R  R          ];
end

