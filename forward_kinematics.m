function [tip_position] = forward_kinematics(S,num_link,numseg,L)

%%%%%%%%
%Calculating the forward kinematics of the actuator based on the given PCC
%parameters, number of links, number of segments, and length of each link
%S: the PCC parameters 
% S(1) = d_L
% S(2) = theta
% S(3) = phi
% num_link = number of links in each segment
% numseg : number of seperate robot segment in the actuator
% L = initial length of each link
%%%%%%%%
i = 1:num_link;
tip_position_1 = zeros(3,numseg);

for j=1:numseg
    alpha = eye(3);
    for k=1:j-1
        alpha = alpha*rotz(S(3*k))*roty(2*num_link*S(3*k-1));
    end
    tip_position_1(:,j)= ((L+S(3*j-2)))*alpha*[cosd(S(3*j))*sum(sind((2*i-1)*S(3*j-1)));...
    sind(S(3*j))*sum(sind((2*i-1)*S(3*j-1)));...
    sum(cosd((2*i-1)*S(3*j-1)))];
end
tip_position = sum(tip_position_1,2);
end