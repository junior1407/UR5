function [x_3, y_3] = computePosition(y_1, dt, polyy)
   if polyy == 0
      polyy = 'poly4' 
   end
   %x_1 and y_1 contain only waypoints.
   total_size = 21.7920;
   temp = total_size/5;
   
   x_1 = [0*temp 1*temp 2*temp 3*temp 4*temp 5*temp];
   f0 = fit(x_1', y_1', polyy);
   
   %x_2 and y_2 are a continuous signal
   x_3 = 0*dt:dt:total_size;
   y_3 = f0(x_3)';
   
 %  %x_3 and y_3 will receive leading and ending zeros to make sure velocity
  % %at the start and end are zero.
 %  x_3 = 0*dt:0.05:8*dt+2*dt;
%   y_3 =[zeros(1,size(0:0.05:dt,2)-1) y_2 zeros(1,size(0:0.05:dt,2)-1)];
  % y_3 = ones(size(y_3));
