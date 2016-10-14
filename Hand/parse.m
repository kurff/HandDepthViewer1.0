% parse
x = textread('position.txt');
y = textread('init_position.txt');
parent = [0 1 2 3 4 1 6 7 8 1 10 11 12 1 14 15 16 1 18 19 20 1 22];
for i = 1:size(x,1)
plot3(x(i,1),x(i,2),x(i,3),'r*');
text(x(i,1),x(i,2),x(i,3),int2str(i));
hold on
plot3(y(:,1),y(:,2),y(:,3),'b*');
text(y(i,1),y(i,2),y(i,3),int2str(i));
end

local = textread('local.txt');
S = trans_reshape(local);

trans = textread('trans.txt');
T = trans_reshape(trans);
S0 = cell(23,1);
S0{1} = T{1};
for i = 2:23
   S0{i} = S0{parent(i)}*T{i};
end
S1 = cell2mat(S0);
dis = S1 - local;
hold on
for i = 1:23
  plotT(S{i})    
    
end

g = textread('global.txt');
G = trans_reshape(g);
% S = cell(23,1);
% S{1} = eye(4);
% for i = 2:23
%    S{i} = S{parent(i)}*T{i};
% end
hold on
for i = 1:23
  plotT(G{i})    
    
end
