load I11.txt
load I22.txt
m1 = I11;
m2 = I22;
 
figure(1);
im1 = imread('I11.jpg');
imshow(im1);
hold on;
plot(m1(:,1), m1(:,2), 'R+', 'LineWidth', 2, 'MarkerSize',10);
hold off;

figure(2);
im2 = imread('I22.jpg');
imshow(im2);
hold on;
plot(m2(:,1), m2(:,2), 'R+', 'LineWidth', 2, 'MarkerSize',10);
hold off;
