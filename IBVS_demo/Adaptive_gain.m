function Lambda = Adaptive_gain(Jp,L0,Linf,L_dot0)
%Adaptive Gain from ViSP
x = norm(Jp,inf);
a = L0 - Linf;
b = L_dot0/a;
c = Linf;
Lambda = a*exp(-b*x)+c;
end