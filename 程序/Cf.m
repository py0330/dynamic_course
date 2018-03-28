function Cf = Cf(vs)
%UNTITLED3 此处显示有关此函数的摘要
%   此处显示详细说明
    w = vs(4:6);
    v = vs(1:3);

    Cf = [
    C3(w)         zeros(3,3)
    C3(v)      C3(w) ];
end

