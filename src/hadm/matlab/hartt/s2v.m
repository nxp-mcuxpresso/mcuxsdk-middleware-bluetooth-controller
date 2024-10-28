function out = s2v(in,fieldname)
out = zeros(1,length(in));
for m = 1:length(in)
    eval(sprintf('out(m) = in(m).%s;',fieldname));
end