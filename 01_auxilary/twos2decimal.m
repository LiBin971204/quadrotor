

function [decimal] = twos2decimal(x)
%  twos2decimal(data,bits)  convert 2s complement to decimal
%                           data - single value or array to convert
%                           bits - how many bits wide is the data (i.e. 8
%                           or 16)
if bitget(x,16) == 1
    decimal = double(int16(bitxor(x,2^16-1)+1)*-1);
else
    decimal = double(int16(x));
end
