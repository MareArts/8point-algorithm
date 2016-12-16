% getCorrectCameraMatrix - Check which of the 4 P solutions from the
%                          essential matrix is the correct one
%
%
% Given the essential matrix, two matching points in two images, and the
% camera calibration of both images, it checks which of the 4 possible
% solutions (p259) is the correct one by reprojecting in 3D.
%
%
% Input  - E   -> 3x3 essential matrix
%        - K1  -> 3x3 Camera calibration of image 1
%        - K2  -> 3x3 Camera calibration of image 2
%        - X   -> 3x2 homogeneous points in images 1 and 2
%
% Output - P   -> 3x4 Correct camera matrix (rotation and translation)
%
%
%
% Author: Isaac Esteban
% IAS, University of Amsterdam
% TNO Defense, Security and Safety
% iesteban@science.uva.nl
% isaac.esteban@tno.nl

function [P] = getCorrectCameraMatrix(PXcam, K1,K2, X)

    % Two matching points in image coordinates (x in image 1 and xp in
    % image 2)
    x = X(:,1);
    xp = X(:,2);
  
    % The first camera matrix is taken P = [I|0] and the other 
    Pcam = [eye(3),zeros(3,1)];
    P = K1*Pcam;
    xhat = inv(K1)*x;
      
    
    % For each camera matrix (Pxcam), reproject the pair of points in 3D
    % and determine the depth in 3D of the point
    % FIRST I DO IT FOR ONE
    X3D = zeros(4,4);
    Depth = zeros(4,2);
    for i=1:4
        
        % First the point is converted
        xphat = inv(K2)*xp;

        % We build the matrix A
        A = [Pcam(3,:).*xhat(1,1)-Pcam(1,:);
             Pcam(3,:).*xhat(2,1)-Pcam(2,:);
             PXcam(3,:,i).*xphat(1,1)-PXcam(1,:,i);
             PXcam(3,:,i).*xphat(2,1)-PXcam(2,:,i)];
        
        % Normalize A
        A1n = sqrt(sum(A(1,:).*A(1,:)));
        A2n = sqrt(sum(A(2,:).*A(2,:)));
        A3n = sqrt(sum(A(1,:).*A(1,:)));
        A4n = sqrt(sum(A(1,:).*A(1,:))); 
        Anorm = [A(1,:)/A1n;
                 A(2,:)/A2n;
                 A(3,:)/A3n;
                 A(4,:)/A4n];
             
        % Obtain the 3D point
        [Uan,San,Van] = svd(Anorm);
        X3D(:,i) = Van(:,end);
        
        % Check depth on second camera
        xi = PXcam(:,:,i)*X3D(:,i);
        w = xi(3);
        T = X3D(end,i);
        m3n = sqrt(sum(PXcam(3,1:3,i).*PXcam(3,1:3,i)));
        Depth(i,1) = (sign(det(PXcam(:,1:3,i)))*w)/(T*m3n);
        
        % Check depth on first camera
        xi = Pcam(:,:)*X3D(:,i);
        w = xi(3);
        T = X3D(end,i);
        m3n = sqrt(sum(Pcam(3,1:3).*Pcam(3,1:3)));
        Depth(i,2) = (sign(det(Pcam(:,1:3)))*w)/(T*m3n);
         
    end;

    % Check which solution is the right one and return
    if(Depth(1,1)>0 && Depth(1,2)>0)
        P = PXcam(:,:,1);
    elseif(Depth(2,1)>0 && Depth(2,2)>0)
        P = PXcam(:,:,2);    
    elseif(Depth(3,1)>0 && Depth(3,2)>0)
        P = PXcam(:,:,3);
    elseif(Depth(4,1)>0 && Depth(4,2)>0)
        P = PXcam(:,:,4);
    end;
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    