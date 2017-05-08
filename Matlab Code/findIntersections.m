% This fucntion takes the feasible payoff matrices from both players and
% the polygon which is the coordinate points for the new equilibrium set
% and the discount factor as parameters. The function computes the operator
% algorithm to calculate the extreme points and use intersectLinePolygon
% function to find the intersections between the extreme points and the
% boundary and the intersections between the two operator.
% Parameters:
% payoff1 is the payoff matrix for player 1
% payoff2 is the payoff matrix for player 2
% polygon is the coordinates of the equilibrium set.
% discountRate is the discount rate
function validIntersections = findIntersections(payoff1,payoff2,polygon,discountRate)
validIntersections = [];
[w1 w2] = operator(payoff1,payoff2,discountRate);
[r c] = size(payoff1);
% w3 is the x-axis for the intersection point
w3 = zeros(r,c);
% w4 is the y-axis for the intersection point
w4 = zeros(r,c);

for i = 1:r
    for j = 1:c
        % calculate h(a) for player 1
        h1 = max(payoff1(:,j))-payoff1(i,j); % h1 is always nonnegative
        % check player 1's IC
        w3(i,j)=((1-discountRate)/discountRate)*h1; % w1 is the continuation value for player 1 and it is always positive
        % calculate h(a) for player 2
        h2 = max(payoff2(i,:))-payoff2(i,j); % h2 is always nonnegative
        % chech player 2's IC
        w4(i,j)=((1-discountRate)/discountRate)*h2; % w2 is the continuation value for player 2 and it is always positive
    end
end

for i = 1:r
    for j = 1:c
        %check if the continuation value exists
        if w1(i,j)~=0
            %create 1-by-4 row vector named line.
            line = [w1(i,j) w1(i,j) 0 1];%In the format [x0 y0 dx dy]
            %intersectLinePolygon returns the intersection points of the
            %lines
            intersection = intersectLinePolygon(line,polygon);
            %If the intersection is not empty, continue the loop
            if ~isempty(intersection)
                %Only the points above and right of the operator
                if intersection(1,2) > w4(i,j)
                    validIntersections(end+1,1) = intersection(1,1);
                    validIntersections(end,2) = intersection(1,2);
                    %Store the coordinates of the original payoff
                    validIntersections(end,3) = payoff1(i,j);
                    validIntersections(end,4) = payoff2(i,j);
                end
                if intersection(2,2) > w4(i,j)
                    validIntersections(end+1,1) = intersection(2,1);
                    validIntersections(end,2) = intersection(2,2);
                    validIntersections(end,3) = payoff1(i,j);
                    validIntersections(end,4) = payoff2(i,j);
                end
            end
        end
        if w2(i,j)~=0
            line = [w2(i,j) w2(i,j) 1 0];
            intersection = intersectLinePolygon(line,polygon);
            if ~isempty(intersection)
                if intersection(1,1) > w3(i,j)
                    validIntersections(end+1,1) = intersection(1,1);
                    validIntersections(end,2) = intersection(1,2);
                    validIntersections(end,3) = payoff1(i,j);
                    validIntersections(end,4) = payoff2(i,j);
                end
                if intersection(2,1) > w3(i,j)
                    validIntersections(end+1,1) = intersection(2,1);
                    validIntersections(end,2) = intersection(2,2);
                    validIntersections(end,3) = payoff1(i,j);
                    validIntersections(end,4) = payoff2(i,j);
                end
            end
        end
    end
end