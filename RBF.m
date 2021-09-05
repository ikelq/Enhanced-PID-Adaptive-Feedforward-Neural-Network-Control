function [ S ] = RBF( Z, Mu,variance,node )
global f;
%RBF Summary of this function goes here
%   Detailed explanation goes here
S=zeros(node,1);
for i =1:node
    S(i,1)=exp(-(Z-Mu(:,i))'*(Z-Mu(:,i))/variance^2);
end

