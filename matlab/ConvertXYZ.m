

% To get the new points' location relative to the beetle
function output = ConvertXYZ(beetle_location,points)

output = points;

for i= 1 : length(points)
    output(i,1) = points(i,1) - beetle_location(1);
    output(i,2) = points(i,2) - beetle_location(2);  
    
end

end