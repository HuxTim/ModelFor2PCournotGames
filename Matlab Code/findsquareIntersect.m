% This function uses the the operator algorithm without the IC constraint
% binding check to find intersection points with the boundary and the
% vertex points.
% Parameters:
% payoff1 is the payoff matrix for player 1
% payoff2 is the payoff matrix for player 2
% discountRate is the discount rate
function [w3 w4] = findsquareIntersect(payoff1,payoff2,discountRate)

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
        % check player 2's IC
        w4(i,j)=((1-discountRate)/discountRate)*h2; % w2 is the continuation value for player 2 and it is always positive
    end
end