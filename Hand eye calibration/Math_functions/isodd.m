function logic = isodd(value)
% Function computes if the value  is odd or even
% e.g.   isodd(3) = true isodd(2) =  false

if rem(value,2) == 1
    logic = true;
else 
    logic = false;
end

end