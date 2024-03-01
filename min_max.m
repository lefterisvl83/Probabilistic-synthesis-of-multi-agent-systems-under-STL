function gamma=min_max(a,E,neg)
%
er=sdpvar(2,1);
F = [E(end).A*er<=E(end).b];
ops = sdpsettings;
obj = -neg*a*er;
optimize(F, obj, ops);
gamma=a*value(er);
%
end