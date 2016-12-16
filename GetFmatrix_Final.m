%//////////////////////////////////////////////////////////////////////////
%// Made by J.H.KIM, 2011 / feelmare@daum.net, feelmare@gmail.com        //
%// blog : http://feelmare.blogspot.com                                  //
%// My Lab : VISLAB(http://me.pusan.ac.kr)                               //
%// Eight-Point Algorithm
%//////////////////////////////////////////////////////////////////////////

clc; clear all; close all;

% Corresponding points between two images
% sample #1 I11.jpg, I22.jpg
%{
load I11.txt; load I22.txt;
m1 = I11; m2 = I22;
%}

%sample #2 I1.jpg, I2.jpg
load I1.txt; load I2.txt;
m1 = I1; m2 = I2;

s = length(m1);
m1=[m1(:,1) m1(:,2) ones(s,1)];
m2=[m2(:,1) m2(:,2) ones(s,1)];
Width = 800; %image width
Height = 600; %image height

% Intrinsic Matrix
load intrinsic_matrix.txt
K = intrinsic_matrix;

% The matrix for normalization(Centroid)
N=[2/Width 0 -1;
    0 2/Height -1;
    0   0   1];

%%
% Data Centroid
x1=N*m1'; x2=N*m2';
x1=[x1(1,:)' x1(2,:)'];  
x2=[x2(1,:)' x2(2,:)']; 

% Af=0 
A=[x1(:,1).*x2(:,1) x1(:,2).*x2(:,1) x2(:,1) x1(:,1).*x2(:,2) x1(:,2).*x2(:,2) x2(:,2) x1(:,1) x1(:,2), ones(s,1)];

% Get F matrix
[U D V] = svd(A);
F=reshape(V(:,9), 3, 3)';
% make rank 2 
[U D V] = svd(F);
F=U*diag([D(1,1) D(2,2) 0])*V';

% Denormalize
F = N'*F*N;
%Verification
%L1=F*m1'; m2(1,:)*L1(:,1); m2(2,:)*L1(:,2); m2(3,:)*L1(:,3);

%%
%Get E
E=K'*F*K;
% Multiple View Geometry 259page
%Get 4 Possible P matrix 
P4 = get4possibleP(E);
%Get Correct P matrix 
inX = [m1(1,:)' m2(1,:)'];
P1 = [eye(3) zeros(3,1)];
P2 = getCorrectCameraMatrix(P4, K, K, inX)

%%
%Get 3D Data using Direct Linear Transform(Linear Triangular method)
Xw = Triangulation(m1',K*P1, m2',K*P2);
xx=Xw(1,:);
yy=Xw(2,:);
zz=Xw(3,:);

figure(1);
plot3(xx, yy, zz, 'r+');


%{
%This code is also run well instead of Triangulation Function.
nm1=inv(K)*m1';
nm2=inv(K)*m2';
% Direct Linear Transform
for i=1:s
    A=[P1(3,:).*nm1(1,i) - P1(1,:);
    P1(3,:).*nm1(2,i) - P1(2,:);
    P2(3,:).*nm2(1,i) - P2(1,:);
    P2(3,:).*nm2(2,i) - P2(2,:)];

    A(1,:) = A(1,:)./norm(A(1,:));
    A(2,:) = A(2,:)./norm(A(2,:));
    A(3,:) = A(3,:)./norm(A(3,:));
    A(4,:) = A(4,:)./norm(A(4,:));

    [U D V] = svd(A);
    X(:,i) = V(:,4)./V(4,4);
end
%}

