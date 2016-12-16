%Find Point in 3D space from given x1,P1,x2,P2
%       [x1] * P1 * Xw = 0
%       [x2] * P2 * Xw = 0
%  Let       [A]*[Xw;Xw] = [0;0]
%  Find [U,D,V] = SVD(A), Xw is last column of V

function Xw = Triangulation(x1,P1,x2,P2)

% Argument
%   x1,x2 = Point in 2D space [x1,...,xn ; y1,...,yn ; 1,...,1]
%   P1,P2 = Projection Transformation Matrix
%   F = Fundamental Matrix
%   Xw = Point in 3D space [X1,...,Xn ; Y1,...,Yn ; Z1,...,Zn ; 1,...,1]

x11 = x1; %- [K1(1,3)*ones(1,size(x1,2)); K1(2,3)*ones(1,size(x1,2)) ; zeros(1,size(x1,2))];
x22 = x2; %- [K2(1,3)*ones(1,size(x2,2)); K2(2,3)*ones(1,size(x2,2)) ; zeros(1,size(x2,2))];


for i=1:size(x11,2)
    %Select point
    sx1 = x11(:,i);
    sx2 = x22(:,i);
    
    
    A1 = sx1(1,1).*P1(3,:) - P1(1,:);
    A2 = sx1(2,1).*P1(3,:) - P1(2,:);
    A3 = sx2(1,1).*P2(3,:) - P2(1,:);
    A4 = sx2(2,1).*P2(3,:) - P2(2,:);
    
    %Set A matric, and find SDV(A)
    A = [A1;A2;A3;A4];
    [U,D,V] = svd(A);
    
    %Point in 3D space is the last column of V
    X_temp = V(:,4);
    X_temp = X_temp ./ repmat(X_temp(4,1),4,1);
    
    Xw(:,i) = X_temp;
    
end


   
   
