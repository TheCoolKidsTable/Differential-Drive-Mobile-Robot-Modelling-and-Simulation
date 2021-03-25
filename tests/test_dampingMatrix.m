%% Main function to generate tests
function tests = test_dampingMatrix
tests = functiontests(localfunctions);
end

%% Test Functions
function testDampingMatrixSymmetric(testCase)
    global param
    nu = [1;2;3;4;5;6];
    D = dampingMatrix(nu,param);
    actual = D.';
    expected = D;
    assertEqual(testCase, actual, expected, 'AbsTol', 1e-12);
end

function testDampingMatrixPositiveSemidefinite(testCase)
    global param
    nu = [1;2;3;4;5;6];
    MRB = dampingMatrix(nu,param);
    actual = min(eig( (MRB + MRB.')/2 ));
    floor = -1e-10;
    assertGreaterThanOrEqual(testCase, actual, floor);
end

%% Optional file fixtures  
function setupOnce(testCase)  % do not change function name
    global param
    addpath ../
    param = robotParameters();
end

function teardownOnce(testCase)  % do not change function name
    clearvars -global param
    rmpath ../
end

%% Optional fresh fixtures  
function setup(testCase)  % do not change function name
end

function teardown(testCase)  % do not change function name
end