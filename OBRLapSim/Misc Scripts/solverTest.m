speed = 6.3;

testCurvatures = linspace(-0.25,0.25,100);
testAccel = zeros(1,length(testCurvatures));


warning('off','MATLAB:illConditionedMatrix');
warning('off','MATLAB:singularMatrix');
warning('off','MATLAB:nearlySingularMatrix');

for i = 1:length(testCurvatures)
    testAccel(i) = solver(Car, speed, testCurvatures(i), 0);
end

warning('on','MATLAB:illConditionedMatrix');
warning('on','MATLAB:singularMatrix');
warning('on','MATLAB:nearlySingularMatrix');

figure
plot(testCurvatures,testAccel)