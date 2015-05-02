function tests = collide_test
tests = functiontests(localfunctions);
end

function assertNoCollision(testcase, map, points)
c = collide(map, points);
verifyEqual(testcase, size(points, 1), length(c(:)));
verifyFalse(testcase, any(c));
end

function assertCollision(testcase, map, points)
c = collide(map, points);
verifyTrue(testcase, isvector(c));
verifyEqual(testcase, size(points, 1), length(c(:)));
verifyTrue(testcase, all(c));
end

function [xyzs] = grid(x, y, z)
[x, y, z] = meshgrid(x, y, z);
xyzs = [x(:), y(:), z(:)];
end

function testEmptyMap(testcase)
map = load_map('emptyMap.txt', 0.2, 2.0, 0.0);
points = grid(0:0.1:10, 0:0.1:10, 0:1.0:6.0);
assertNoCollision(testcase, map, points);
end

function testSingleCube(testcase)
map = load_map('singleCube.txt', 0.1, 1.0, 0.0);
top_points    = grid(0:0.1:10, 0:0.1:10, 4.1:0.4:6);
left_points   = grid(0:0.2:4.4, 0:0.2:10, 0:0.4:6);
right_points  = grid(5.6:0.2:10, 0:0.2:10, 0:0.4:6);
bottom_points = grid(0:0.2:10, 0:0.2:10, 0:0.4:2.2);
square_points = grid(4.6:0.2:5.4, 4.6:0.2:5.4, 2.5:0.2:3.5);

assertNoCollision(testcase, map, top_points);
assertNoCollision(testcase, map, bottom_points);
assertNoCollision(testcase, map, left_points);
assertNoCollision(testcase, map, right_points);
assertCollision(testcase, map, square_points);
end

function testMap1(testcase)
map = load_map('map0.txt', 0.2, 0.5, 0.2);

valid = [0.0  -1.0 2.0; 
         3.0  17.0 4.0; 
         0.0  -5.0 0.5];
collision = [0.0 2.0 1.0; 
             3.0 18.5 4.5];
assertNoCollision(testcase, map, valid);
assertCollision(testcase, map, collision);
end
