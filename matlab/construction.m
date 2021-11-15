raw = readtable('table.csv','ReadVariableNames',false);
rawM = raw{1:size(raw),:};
nrPoint = length(rawM)/5;
for i =1: nrPoint
   T(i,1) =  mapfun(rawM(5*i-4,1)*256+rawM(5*i-3,1),0,4096,0,2*pi);
   T(i,2) =  mapfun(rawM(5*i-2,1)*256+rawM(5*i-1,1),1024,3072,0,pi);
   T(i,3) =  rawM(5*i,1);

end

%% 



t=1;
for i= 1 : length(T)
%    [F(i,1),F(i,2)]=pol2cart(T(i,1),T(i,3)*sin(T(i,2))+7.5*sin(pi-T(i,2))+5*cos(pi-T(i,2)));
%    F(i,3)=16+7.5*cos(pi-T(i,2))-5*sin(pi-T(i,2))-cos(T(i,2))*T(i,3);
   [F(i,1),F(i,2)]=pol2cart(T(i,1),T(i,3)*sin(T(i,2)));
    F(i,3)=16-cos(T(i,2))*T(i,3);

       
end


ptCloud = pointCloud(F);
pcshow(ptCloud);
model = pcfitcuboid(ptCloud);
plot(model);

%% get the top left point angle
x1=165;
y1=420;
z1=134;
[a1,m1]=cart2pol(x1,y1);
n1=16-z1;
r1=sqrt(m1*m1+n1*n1);
b1=acos(n1/r1);
panAngle = mapfun(a1,0,2*pi,0,4096);
if panAngle>2048 
    panAngle=panAngle-2048;
else
    panAngle=panAngle+2048;
end

panAngleH=floor(panAngle/256);
panAngleL=round(panAngle-panAngleH*256);
tiltAngle = mapfun(b1,0,pi,1024,3072);
tiltAngle=4096-tiltAngle;
tiltAngleH =floor(tiltAngle/256);
tiltAngleL=round(tiltAngle-tiltAngleH*256);
