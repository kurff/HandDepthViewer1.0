function plotT(T)
  pt = T(1:3,4);
  R = T(1:3,1:3);
  plotax(pt,R,10);

end