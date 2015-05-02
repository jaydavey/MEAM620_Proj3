%test distance
lpath = 0;
lpathAStar = 0;
for i=1 : size(path,1)-1
    d = sqrt((path(i+1,1)-path(i,1))^2+(path(i+1,2)-path(i,2))^2+(path(i+1,3)-path(i,3))^2);
    lpath = lpath + d;
    
    
end

for i=1 : size(pathAStar,1)-1
    d = sqrt((pathAStar(i+1,1)-pathAStar(i,1))^2+(pathAStar(i+1,2)-pathAStar(i,2))^2+(pathAStar(i+1,3)-pathAStar(i,3))^2);
    lpathAStar = lpathAStar + d;
    
    
end