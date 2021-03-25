%% Main function to generate tests
function tests = test_rigidBodyCoriolis
tests = functiontests(localfunctions);
end

%% Test Functions
function testCoriolisSkewSymmetric1(testCase)
    global param
    nu = [1;2;3;4;5;6];
    CRB = rigidBodyCoriolisMatrixVelocity(nu,param);
    actual = -CRB.';
    expected = CRB;
    assertEqual(testCase, actual, expected, 'AbsTol', 1e-12);
end

function testCoriolisSkewSymmetric2(testCase)
    p = [1;2;3;4;5;6];
    CRB = rigidBodyCoriolisMatrixMomentum(p);
    actual = -CRB.';
    expected = CRB;
    assertEqual(testCase, actual, expected, 'AbsTol', 1e-12);
end

function testDAlembertForces(testCase)
    global param
    nu = [1;2;3;4;5;6];
    MRB = rigidBodyMassMatrix(param);
    p = MRB*nu;
    CRBnu = rigidBodyCoriolisMatrixVelocity(nu,param);
    CRBp = rigidBodyCoriolisMatrixMomentum(p);
    actual = CRBp*nu;
    expected = CRBnu*nu;
    assertEqual(testCase, actual, expected, 'AbsTol', 1e-12);
end


%% Optional file fixtures  
function setupOnce(testCase)  % do not change function name
    global param
    param.m = 42; 
	param.IBb = diag([4;5;6]);
	param.rCBb = [1;2;3]/100;
    addpath ../
end

function teardownOnce(testCase)  % do not change function name
    rmpath ../
    clearvars -global param
end

%% Optional fresh fixtures  
function setup(testCase)  % do not change function name
end

function teardown(testCase)  % do not change function name
end