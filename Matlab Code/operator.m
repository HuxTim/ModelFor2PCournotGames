% This function takes the payoff matrix for player 1 and player 2 and
% discount rate as parameters and generates the deviation point matrices
% and the intersection points matrices for the players. By implementing
% this function on a specific action profile, it gives back either
% horizontal lines or vertical lines of the operator for each profile.
% Parameters:
% payoff1 is the payoff matrix for player 1
% payoff2 is the payoff matrix for player 2
% discountRate is the discount rate
function [w1 w2] = operator(payoff1,payoff2,discountRate)

% Get the number of each rows and columns from the payoff matrix
% Both players share the same number of payoff options
[r c] = size(payoff1);

% Construct zero matrice for the operator values.
w1 = zeros(r,c);
w2 = zeros(r,c);
% Loop through each variables in the matrix for player 1
for i = 1:r
    for j = 1:c
        % Compute the deviation point, which is the difference between
        % the current payoff option and the maximum payoff option
        h1 = max(payoff1(:,j))-payoff1(i,j); % deviation value is always positive
        % Check the operator condition for player 1's Incentive Constraints
        % If the product of the deviation value and the discount rate is
        % higher than the product between the payoff option and 1 minus
        % discount rate, compute for the boundary intersection point
        if payoff1(i,j)*discountRate <= (1-discountRate)*h1
            % Store the continuation value into the w matrix with the 
            % deviation point multiplying with (1-d)/d
            w1(i,j)=((1-discountRate)/discountRate)*h1; 
            % w1 is the continuation value for player 1 and is always positive
        end
        % Compute deviation matrix for player 2
        h2 = max(payoff2(i,:))-payoff2(i,j); 
        % Check condition for player 2
        if payoff2(i,j)*discountRate <= (1-discountRate)*h2
            %Compute continuation value for player 2
            w2(i,j)=((1-discountRate)/discountRate)*h2; 
        end
    end
end
end


            
            

