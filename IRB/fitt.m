function y = fitt(x)
   a=x(1);
   b=x(2);
   c=x(3);
   d=x(4);
   E=x(5);
   f=x(6);
   g=x(7);
   h=x(8);
   
   K_1 = [a         0         0         0
                     0   b         0         0;
                     0        0   c         0;
                     0         0         0   d];
    K_0 = [  E         0         0         0;
                     0   f         0         0;
                     0         0  g         0;
                     0         0         0   h];
    simOut = sim('control2_irb');
    y = sum(simOut.erro2.data(:,1).^2) + sum(simOut.erro2.data(:,2).^2) + sum(simOut.erro2.data(:,3).^2) + sum(simOut.erro2.data(:,4).^2);
                
