function [x_v, y_v,x_a, y_a] = calcVelocityAcceleration(x_s, y_s)
    x_v = x_s(1:size(x_s,2)-1);
    x_a = x_s(1:size(x_s,2)-2);
    y_v = zeros(1, size(x_s,2)-1);
    for i = 1:size(x_s,2)-1
        y_v(i) = (y_s(i+1) - y_s(i))/ (x_s(i+1) - x_s(i));
    end
    y_a = zeros(1, size(x_s,2)-2);
    for i = 1:size(x_s,2)-2
        y_a(i) = (y_v(i+1) - y_v(i))/ (x_s(i+1) - x_s(i));
    end
end