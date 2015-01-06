function DrawFrame( data, x,y,z,i , linewidth, style)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

line([data(i,2) data(i,2)+x(1,1,i)],[data(i,3) data(i,3)+x(2,1,i)],[data(i,4) data(i,4)+x(3,1,i)],'LineWidth',linewidth ,'Color','r','LineStyle',style);
line([data(i,2) data(i,2)+y(1,1,i)],[data(i,3) data(i,3)+y(2,1,i)],[data(i,4) data(i,4)+y(3,1,i)],'LineWidth',linewidth ,'Color','g','LineStyle',style);
line([data(i,2) data(i,2)+z(1,1,i)],[data(i,3) data(i,3)+z(2,1,i)],[data(i,4) data(i,4)+z(3,1,i)],'LineWidth',linewidth ,'Color','b','LineStyle',style);


end

