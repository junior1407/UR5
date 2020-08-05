function Transf = getTransformationMatrix(pos, orient)
    Transf=eye(4,4);
    Transf(1:3,1:3)=orient;
    Transf(1:3,4)=pos;
end