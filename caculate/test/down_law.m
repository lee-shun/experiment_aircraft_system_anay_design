clc
clear
x_max=400;
y_max=3.6;
deta_x=10;
arry_x=0:deta_x:x_max;
[m n]=size(arry_x);
for i=1:n
    arry_y(i)=y_max*(0.5+0.5*cos(pi*arry_x(i)/x_max));
end
plot(arry_x,arry_y);
%Ð´ÈëÎÄ¼þ
fo=fopen('down_law.txt','w');
fprintf(fo,'%s','# x y');
fprintf(fo,'\n');
for i=1:n
    fprintf(fo,'%f   ',arry_x(i));
    fprintf(fo,'%f   ',arry_y(i));
    fprintf(fo,'\n');
end
fclose(fo);