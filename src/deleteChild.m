% Delete elements.
function A = deleteChild(A,a)
[~,idx_a] = find(A==a);
A(idx_a) = [];
end
