k = 0.1;
a = 1;
b = 1;
c = 1;
h = k * tf([1 1],[1 a b c])
h = feedback(h,1)
step(h)