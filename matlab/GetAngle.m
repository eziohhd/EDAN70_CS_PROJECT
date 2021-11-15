% Get the angle relationship of the point relative to the beetle
function angle_distance = GetAngle(points)

angle_distance = zeros(length(points),3);


for i= 1 : length(points)
    
%     x = points(i,1);
%     y = points(i,2);
%     z = points(i,3);   
%     angle_distance(i,1) = atan(sqrt(x*x+y*y)/z);
%     angle_distance(i,2) = atan(y/x);
    [angle_distance(i,1),angle_distance(i,2),angle_distance(i,3)] = ...
     cart2sph(points(i,1),points(i,2),points(i,3));
    
end

end