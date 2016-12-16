function P4 = get4possibleP(E)

% SVD of E
[U,S,V] = svd(E);

%W
W = [0,-1,0;1,0,0;0,0,1];        

P4 = zeros(3,4,4);
R1 = U*W*V';
R2 = U*W'*V';
T1 = U(:,3);
T2 = -U(:,3);

P4(:,:,1) = [R1, T1];
P4(:,:,2) = [R1, T2];
P4(:,:,3) = [R2, T1];
P4(:,:,4) = [R2, T2];
  
end

