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
direction = [45/180*pi,30/180*pi];%The angle relationship between the laser and the beetle that we want
beetle_location = [120,300]; % Detect the beetle location
new_points = ConvertXYZ(beetle_location,F); % Re-establish the coordinate system with the beetle as the coordinate origin 
angle_distance = GetAngle(new_points);% Conversion of Cartesian Coordinate System to Spherical Coordinate System
laser_target = GetLaserTarget(direction,angle_distance,F);% Get the laser target


% [azimuth,elevation,r] = cart2sph(laser_target(1),laser_target(2),laser_target(3));
% location = [azimuth,elevation,r];
% ptCloud = pointCloud(F);
% pcshow(ptCloud);
% model = pcfitcuboid(ptCloud);
% plot(model);

