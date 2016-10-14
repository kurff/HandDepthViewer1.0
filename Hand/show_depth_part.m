% show depth and part
for i = 1:100
   depth = imread(['./depth/',int2str(i),'.png']);
   orient = imread(['./orient/',int2str(i),'.png']);
   part = imread(['./part/',int2str(i),'.png']);
   figure(1);
   imagesc(depth);
   figure(2);
   imagesc(orient);
   figure(3);
   imagesc(part);
   drawnow
   
end