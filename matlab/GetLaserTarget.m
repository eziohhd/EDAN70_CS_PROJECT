function output = GetLaserTarget(direction,angle_distance,data_from_lidar)

minimum = 100000; % a large number
for i= 1 : length(angle_distance)
    
    find_minimum = abs(angle_distance(i,1)-direction(1)) + abs(angle_distance(i,2)-direction(2));

    if (minimum > find_minimum )   
        minimum = find_minimum;
        output = data_from_lidar(i,:);
    end   
end
end