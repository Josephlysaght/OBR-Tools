function curvature = calcCurvature(X,Y)
%This function calculates the corner radius at each point in a track.
%It uses the current, previous and following coordinates to find the radius
%of the circle that pass through these three points using geometric relations

%Creating vector cornerradius
curvature = zeros(length(X),1);
YawSign = curvature;

if X(1) == X(end) && Y(1) == Y(end)

    a = sqrt((X(2)-X(end))^2+(Y(2)-Y(end))^2);
    b = sqrt((X(2)-X(1))^2+(Y(2)-Y(1))^2);
    c = sqrt((X(1)-X(end))^2+(Y(1)-Y(end))^2);
    A = abs(acos((b^2+c^2-a^2)/(2*b*c)));

    curvature(1) = (2*sin(pi-A))/a;

    
    a = sqrt((X(1)-X(end-1))^2+(Y(1)-Y(end-1))^2);
    b = sqrt((X(1)-X(end))^2+(Y(1)-Y(end))^2);
    c = sqrt((X(end)-X(end-1))^2+(Y(end)-Y(end-1))^2);
    A = abs(acos((b^2+c^2-a^2)/(2*b*c)));

    curvature(end) = (2*sin(pi-A))/a;
    
else
    
    curvature(1) = 0;
    curvature(end) = 0;

end

YawSign(1) = (Y(1) - Y(end))/(X(1) - X(end));
YawSign(end) = (Y(end) - Y(end-1))/(X(end) - X(end-1));

%I have used the method suggested by James Hakewill. The pdf can be easily found
%on google just by typing "laptime james hakewill".
for n = 2:length(X)-1
    
    YawSign(n) = (Y(n) - Y(n-1))/(X(n) - X(n-1));
    a = sqrt((X(n+1)-X(n-1))^2+(Y(n+1)-Y(n-1))^2);
    b = sqrt((X(n+1)-X(n))^2+(Y(n+1)-Y(n))^2);
    c = sqrt((X(n)-X(n-1))^2+(Y(n)-Y(n-1))^2);
    A = abs(acos((b^2+c^2-a^2)/(2*b*c)));

    curvature(n) = (2*sin(pi-A))/a;
    
end

YawDer = gradient(YawSign);
curvature = sign(YawDer).*curvature;