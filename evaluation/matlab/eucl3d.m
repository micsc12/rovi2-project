function[dist] = eucl3d(x,y) 
    dist = sqrt((y(:,1)-x(:,1)).^2+(y(:,2)-x(:,2)).^2+(y(:,3)-x(:,3)).^2);
end