function T = trans_reshape(trans)
 T = cell(1,23);
 for i = 1:23
     T{i} = trans((i-1)*4+1:i*4,:);
     
 end


end