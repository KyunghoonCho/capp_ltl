% Initialize the tree structure.
%   Input
%       x_init: an initial physical state
%       q_init: an initial automaton state
%       MAXSIZE: a maximum number of nodes
%       D: discrete region
%       index_RI: index of 'region of interest'
%       cost_type: type of cost
function T = initializeTree(x_init,q_init,MAXSIZE,D,index_RI,map,cost_type)

T.MAXSIZE = MAXSIZE;

T.v.x = zeros(size(x_init,1),MAXSIZE);
T.v.d = zeros(1,MAXSIZE);
T.v.alpha = zeros(1,MAXSIZE);
T.v.nsel = zeros(1,MAXSIZE);
T.v.cost = zeros(1,MAXSIZE);
T.v.cost_p = zeros(1,MAXSIZE);
T.v.path = cell(1,MAXSIZE);
T.v.pseg = cell(1,MAXSIZE);
T.v.pseg_cost = zeros(1,MAXSIZE);

T.e.u       = cell(1,MAXSIZE);
T.e.parent  = zeros(1,MAXSIZE);
T.e.rewire  = zeros(1,MAXSIZE);
T.e.child   = cell(1,MAXSIZE);

% do initialization
T.num_node = 1;
T.v.x(:,1) = x_init; % physical state
T.v.d(1,1) = findIndex(D,index_RI,x_init);
T.v.alpha(1,1) = q_init; % automaton state
T.v.nsel(1,1) = 0;
c = calculateCostP(x_init,map,cost_type);
T.v.cost(1,1) = c;
T.v.cost_p(1,1) = c;
T.v.path{1,1} = x_init;
T.v.pseg{1,1} = x_init;

end
