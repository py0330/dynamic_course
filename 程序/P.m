function pm = P(vs)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明

v=vs(1:3);
w=vs(4:6);

if(norm(w)<1e-10)
    pm=[
        eye(3)    ,v(:)
        zeros(1,3),1];
else
    % 归一化
    theta = norm(w);
    v = v(:)/theta;
    w = w(:)/theta;
    
    R = eye(3) + sin(theta)*C3(w) + (1-cos(theta))*C3(w)*C3(w);
    p = (eye(3)-R)*C3(w)*v + theta * w' * v * w;

    pm=[
     R          , p 
     zeros(1,3) , 1   ];
end

end

