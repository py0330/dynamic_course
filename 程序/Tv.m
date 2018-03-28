function Tv = Tv(pm)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明

R = pm(1:3,1:3);
p = pm(1:3,4); 

Tv = [   
    R           C3(p)*R  
    zeros(3,3)  R          ];
end

