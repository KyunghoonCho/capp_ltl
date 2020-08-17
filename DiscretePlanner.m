%% Discrete (High-Level) Planner

% Reference
%   [1] Bhatia, Amit, Lydia E. Kavraki, and Moshe Y. Vardi.
%   "Sampling-based motion planning with temporal goals."
%   Robotics and Automation (ICRA), 2010 IEEE International Conference on. IEEE, 2010.
%
%   [2] Plaku, Erion.
%   "Planning in discrete and continuous spaces: From LTL tasks to robot motions."
%   Advances in Autonomous Robotics. Springer Berlin Heidelberg, 2012. 331-342.
%
%   [3] McMahon, James, and Erion Plaku.
%   "Sampling-based tree search with discrete abstractions for motion planning with dynamics and temporal logic."
%   Intelligent Robots and Systems (IROS 2014), 2014 IEEE/RSJ International Conference on. IEEE, 2014.

classdef DiscretePlanner < handle
    properties
        points;             % 2D point-array
        RI;                 % region of interest
        Obs;                % obstacle
        RI_margin;          % margin for region of interest
        
        D;                  % discrete region
        num_D;              % number of discrete region (D).
        index_RI;           % start index of region of interest
        Transition;         % transition relation
        area_D;             % area of discrete region
        edgecost_D;         % edge cost of discrete system
        
        nbaParse;           % NBA of LTL spec
        ap_ltlspec;         % atomic propositions of LTL specs
        ap_origin_length;   % number of atomic propostions of LTL specs
        tranZ;              % transition info of automaton states
        w_z;                % distance between automaton states and accepting states
        w_zd;               % distance between states in the product automaton
        prePath_z;          % pre-computed path;
        prePath_g;          % pre-computed path;
        
        use_mex;            % whether to use mex-function;
        
        %% Abstraction
        GRAPH;              % Graph in product space (len_G x len_G matrix)
        C_GRAPH;            % cost of edge in GRAPH
        num_G;              % number of states in DA.GRAPH
        num_Z;              % number of NBA states (A.Z)
        
    end
    
    methods
        %% Constructor
        function obj = DiscretePlanner(P,nbaParse,AP_LTLSPEC,RI_margin,use_mex)
            % STEP 1: update workspace information
            N = size(P,2);
            obj.RI = cell(1,N-1);
            ind_RI = 0;
            obj.Obs = cell(1,N-1);
            ind_obs = 0;
            for nidx_n = 1:1:N
                if(nidx_n>=2)
                    if(P{nidx_n}.label~='o')
                        ind_RI = ind_RI+1;
                        obj.RI{ind_RI}.p        = P{nidx_n}.points;
                        obj.RI{ind_RI}.label    = P{nidx_n}.label;
                    else
                        ind_obs = ind_obs+1;
                        obj.Obs{ind_obs}.p = P{nidx_n}.points;
                        obj.Obs{ind_obs}.label = P{nidx_n}.label;
                    end
                end
                obj.points = [obj.points; P{nidx_n}.points];
            end
            obj.RI = obj.RI(1,1:ind_RI);
            obj.RI_margin = RI_margin;
            obj.Obs = obj.Obs(1,1:ind_obs);
            
            % STEP 2: update automaton (LTL) information
            obj.nbaParse = nbaParse;
            
            % add atomic propositions
            obj.ap_ltlspec = AP_LTLSPEC; % {'a', 'b', 'c'};
            obj.ap_origin_length = size(AP_LTLSPEC,2);
            
            % add (negative) atomic propositions  % {'a', 'b', 'c', '!a', '!b', '!c'};
            for nidx_ap = 1:1:size(AP_LTLSPEC,2)
                ap_neg = ['!' AP_LTLSPEC{1,nidx_ap}];
                obj.ap_ltlspec = [obj.ap_ltlspec ap_neg];
            end
            
            % add '1'
            obj.ap_ltlspec = [obj.ap_ltlspec '1'];
            
            % transition info in automaton states is updated
            obj = DecomposeNBA(obj);
            
            % whether to use mex-function
            obj.use_mex = use_mex;
        end
        
        function obj = DecomposeNBA(obj)
            obj.tranZ = cell(obj.nbaParse.n_states,obj.nbaParse.n_states);
            
            for n_index_from = 1:1:size(obj.nbaParse.adj,1)                 % nbaParse.adj rows
                for n_index_to = 1:1:size(obj.nbaParse.adj,2)               % nbaParse.adj columns
                    tr_nba = obj.nbaParse.adj{n_index_from,n_index_to};     % nbaParse.adj{1,1}, {'a'&&'!o'}, {'s'}||{'!e'}
                    % one thing to notice: {'a1'&&'!o1'} -> 1x1 cell
                    %                      {'s1'}||{'!e1'} -> 1x2 cell
                    % check whether or statement is included
                    if(size(tr_nba,2)>1)
                        obj.tranZ{n_index_from,n_index_to} = cell(1,size(tr_nba,2));
                        is_OR = 1;
                    else
                        is_OR = 0;
                    end
                    
                    % update transition info related with automaton state.
                    for nidx_tr = 1:1:size(tr_nba,2)
                        alpha_nba = tr_nba{1,nidx_tr};                  % nbaParse.adj{1,1}{1,1}, 'a' '!o', 's' '!e'
                        for nap = 1:1:size(alpha_nba,2)
                            ap = alpha_nba{nap};                        % nbaParse.adj{1,1}{1,1}{1}, a, !e
                            if(is_OR)
                                obj = update_z_tran(obj,n_index_from,n_index_to,nidx_tr,ap);
                            else
                                obj = update_z_tran(obj,n_index_from,n_index_to,0,ap);
                            end
                        end % for_nap
                    end % for_ntr
                end % for_ntj
            end % for_n_nindex
        end
        
        function obj = update_z_tran(obj,n_index_from,n_index_to,index_tz,ap)
            ap_array = obj.ap_ltlspec;
            for nidx_ap = 1:1:size(ap_array,2)
                if(strcmp(ap,ap_array{nidx_ap}))
                    if(index_tz==0)
                        obj.tranZ{n_index_from,n_index_to}=sort([obj.tranZ{n_index_from,n_index_to} nidx_ap]);
                    else
                        obj.tranZ{n_index_from,n_index_to}{1,index_tz}=sort([obj.tranZ{n_index_from,n_index_to}{1,index_tz} nidx_ap]);
                    end
                end
            end
        end
        
        %% Naive decomposition
        function obj = naive_decomposition(obj,P,sp)
            obj.index_RI = 3;
            obj.num_D   = 2+size(obj.RI,2);
            obj.D       = cell(1,obj.num_D);
            
            for nidx_i=obj.index_RI:1:obj.num_D
                psel                = obj.RI{nidx_i-obj.index_RI+1}.p;
                obj.D{nidx_i}.p     = psel;
                obj.D{nidx_i}.p_range = [(min(psel,[],1)' + obj.RI_margin*ones(size(psel,2),1)) ...
                    (max(psel,[],1)' - obj.RI_margin*ones(size(psel,2),1))];
                obj.D{nidx_i}.area  = 1;
                obj.D{nidx_i}.label = obj.RI{nidx_i-obj.index_RI+1}.label;
            end
            
            obj.D{1}.p = sp';
            obj.D{1}.area = 1; % randomly assigned value
            obj.D{1}.label = 'w';
            
            obj.D{2}.p = P{1}.points;
            obj.D{2}.area = 1; % randomly assigned value
            obj.D{2}.label = 'w';
            
            % update transition
            obj.Transition = cell(1,obj.num_D);
            obj.Transition{1,1} = 2;
            obj.Transition{1,2} = obj.index_RI:obj.num_D;
            for nidx_t = obj.index_RI:1:obj.num_D
                obj.Transition{1,nidx_t} = 2;
            end
            
            % update 'area_D'
            obj.area_D = zeros(1,obj.num_D);
            for nidx_i = 1:1:obj.num_D
                obj.area_D(1,nidx_i) = obj.D{nidx_i}.area;
            end
        end
        
        %% Triangulation decomposition
        function triangulation(obj,P,path_to_triangulate,path_to_triangle,triangle_type)
            if(triangle_type == 1)
                % http://www.cs.unc.edu/~dm/CODE/GEM/chapter.html#FIG
                % run 'Fast Polygon Triangulation based on Seidel's Algorithm',
                % and load its result to MATLAB.
                
                fName = 'discreteAbstraction';
                createFileTriangulate(P,fName);
                
                [s, triangulate_output] = system([path_to_triangulate,' ',fName] );
                if s ~= 0
                    error('Error when applying triangulate.')
                end
                
                % parse triangulate output
                triangulate_output = textscan(triangulate_output, '%s', 'delimiter', '\n');
                triangulate_output = triangulate_output{1};
                
                % update discrete region info
                size_RI         = size(obj.RI,2);
                obj.index_RI    = size(triangulate_output, 1) + 1;
                obj.D           = cell(1,size(triangulate_output, 1)+size_RI);
                obj.num_D       = size(obj.D,2);
                
                % non-RI regions
                for nidx_i=1:size(triangulate_output,1)
                    line = triangulate_output{nidx_i, 1};
                    line_separated = strsplit(line,' ');
                    obj.D{nidx_i}.p = [obj.points(str2double(line_separated{3}),:); ...
                        obj.points(str2double(line_separated{4}),:); obj.points(str2double(line_separated{5}),:)];
                    obj.D{nidx_i}.area = polyarea(obj.D{nidx_i}.p(:,1)',obj.D{nidx_i}.p(:,2)');
                    obj.D{nidx_i}.label = 'w';
                end
                
                % RI regions
                for nidx_i = obj.index_RI:1:size(triangulate_output, 1)+size_RI
                    psel            = obj.RI{nidx_i-size(triangulate_output,1)}.p;
                    obj.D{nidx_i}.p     = psel;
                    obj.D{nidx_i}.p_range = [(min(psel,[],1)' + obj.RI_margin*ones(size(psel,2),1)) ...
                        (max(psel,[],1)' - obj.RI_margin*ones(size(psel,2),1))];
                    obj.D{nidx_i}.area  = polyarea(psel(:,1)',psel(:,2)');
                    obj.D{nidx_i}.label = obj.RI{nidx_i-size(triangulate_output,1)}.label;
                end
            else
                % https://www.cs.cmu.edu/~quake/triangle.html
                % ref
                %   Shewchuck, J. R.
                %   "Engineering a 2D quality mesh generator and Delaunay triangulator Applied Computational Geometry:
                %   Towards Geometric Engineering vol 1148 ed MC Lin and D Manocha." (1996).
                
                fName = 'discreteAbstraction';
                createPolyFile(P,[fName '.poly']);
                
                switch triangle_type
                    case 2
                        system([path_to_triangle,' -a0.1 -q -pc ',fName,'.poly'] );
                    case 3
                        system([path_to_triangle,' -a0.2 -q -pc ',fName,'.poly'] );
                    case 4
                        system([path_to_triangle,' -a0.4 -q -pc ',fName,'.poly'] );
                    case 5
                        system([path_to_triangle,' -a0.8 -q -pc ',fName,'.poly'] );
                    case 6
                        system([path_to_triangle,' -a1.6 -q -pc ',fName,'.poly'] );
                    case 7
                        system([path_to_triangle,' -a2.4 -q -pc ',fName,'.poly'] );
                    case 8
                        system([path_to_triangle,' -a3.2 -q -pc ',fName,'.poly'] );
                    case 9
                        system([path_to_triangle,' -a3.6 -q -pc ',fName,'.poly'] );
                    case 10
                        system([path_to_triangle,' -a4.0 -q -pc ',fName,'.poly'] );
                end
                
                % read node result
                node_filename = [fName '.1.node'];
                fileID  = fopen(node_filename,'r');
                S       = textscan(fileID,'%s','delimiter','\n');
                
                % read 1st line
                line_1st = S{1}{1,1};
                line_separated = strsplit(line_1st,' ');
                num_vertex = str2double(line_separated{1,1});
                dim = str2double(line_separated{1,2});
                
                vertices = zeros(num_vertex,dim);
                for nidx_line = 2:1:(num_vertex+1)
                    line_read = S{1}{nidx_line,1};
                    line_separated = strsplit(line_read,' ');
                    
                    point_tmp = zeros(1,dim);
                    for nidx_dim = 2:1:(dim+1)
                        point_tmp(1,nidx_dim-1) = str2double(line_separated{1,nidx_dim});
                    end
                    vertices((nidx_line-1),1:dim)=point_tmp;
                end
                
                % read edge result
                edge_filename = [fName '.1.ele'];
                fileID = fopen(edge_filename,'r');
                S = textscan(fileID,'%s','delimiter','\n');
                
                % read 1st line
                line_1st = S{1}{1,1};
                line_separated = strsplit(line_1st,' ');
                num_triangle = str2double(line_separated{1,1});
                num_vertexPerTriangle = str2double(line_separated{1,2});
                
                % update discrete region info
                size_RI = size(obj.RI,2);
                obj.index_RI = num_triangle + 1;
                obj.D = cell(1,num_triangle+size_RI);
                obj.num_D = size(obj.D,2);
                for nidx_line = 2:1:(num_triangle+1)
                    line_read = S{1}{nidx_line,1};
                    line_separated = strsplit(line_read,' ');
                    
                    point_tmp = zeros(num_vertexPerTriangle,dim);
                    for nidx_nv = 2:1:(num_vertexPerTriangle+1)
                        point_tmp(nidx_nv-1,:) = vertices(str2double(line_separated{1,nidx_nv}),:);
                    end
                    
                    obj.D{1,nidx_line-1}.p      = point_tmp;
                    obj.D{1,nidx_line-1}.area   = polyarea(point_tmp(:,1)',point_tmp(:,2)');
                    obj.D{1,nidx_line-1}.label  = 'w';
                end
                
                % RI regions
                for nidx_i=obj.index_RI:1:num_triangle+size_RI
                    psel            = obj.RI{nidx_i-num_triangle}.p;
                    obj.D{nidx_i}.p     = psel;
                    obj.D{nidx_i}.p_range = [(min(psel,[],1)' + obj.RI_margin*ones(size(psel,2),1)) ...
                        (max(psel,[],1)' - obj.RI_margin*ones(size(psel,2),1))];
                    obj.D{nidx_i}.area  = polyarea(psel(:,1)',psel(:,2)');
                    obj.D{nidx_i}.label = obj.RI{nidx_i-num_triangle}.label;
                end
            end
            
            % update 'area_D'
            obj.area_D = zeros(1,obj.num_D);
            for nidx_i = 1:1:obj.num_D
                obj.area_D(1,nidx_i) = obj.D{nidx_i}.area;
            end
            
        end
        
        %% Make Transition
        function obj = make_transition(obj)
            % update transition info in the form of adjacency lists.
            sizeD = size(obj.D,2);
            obj.Transition = cell(1,sizeD);
            for nt = 1:1:sizeD
                list = check_connection(obj,sizeD,nt);
                obj.Transition{1,nt} = list;
                if(isempty(list))
                end
            end
        end
        
        function list = check_connection(obj,sizeD,index)
            list = zeros(1,sizeD);
            index_list = 0;
            
            points_sel = obj.D{index}.p;
            
            for nt=1:1:sizeD
                if(nt==index)
                    continue;
                end
                
                points_others = obj.D{nt}.p;
                
                count = 0;
                for nsel = 1:1:size(points_sel,1)
                    p_sel = points_sel(nsel,:);
                    diff  = repmat(p_sel,size(points_others,1),1) - points_others;
                    dist_vec = sum(abs(diff),2);
                    [idx_found,~] = find(dist_vec < 0.001);
                    if(~isempty(idx_found))
                        count = count + 1;
                    end
                    if(count == 2)
                        break;
                    end
                end
                
                if(count == 2)
                    index_list = index_list + 1;
                    list(1,index_list) = nt;
                end
            end
            list = list(1,1:index_list);
        end
        
        %% Compute edge_cost
        function obj = compuete_edgeCost(obj,R,map,param)
            obj.edgecost_D = zeros(obj.num_D,obj.num_D);
            for nrow = 1:1:obj.num_D
                pose_row = sum(obj.D{1,nrow}.p,1)/size(obj.D{1,nrow}.p,1);
                for ncol = 1:1:obj.num_D
                    pose_col    = sum(obj.D{1,ncol}.p,1)/size(obj.D{1,ncol}.p,1);
                    pseg        = [pose_row' pose_col'];
                    %cost_pseg = calculateCost(pseg,map,param.interval_cost,param.cost_type,0);
                    cost_pseg   = calculateCost(pseg,map,param.interval_cost,'distance',R,0);
                    obj.edgecost_D(nrow,ncol) = cost_pseg;
                end
            end
        end
        
        function obj = compute_C_GRAPH(obj)
            % compute cost of edge in the product graph
            L = zeros(obj.num_G,obj.num_G);
            for nrow = 1:1:obj.num_G
                nidx_d_row = floor((nrow-1)/obj.num_Z) + 1;
                for ncol = 1:1:obj.num_G
                    nidx_d_col      = floor((ncol-1)/obj.num_Z) + 1;
                    L(nrow,ncol)    = obj.edgecost_D(nidx_d_row,nidx_d_col);
                end
            end
            
            obj.C_GRAPH = L;
        end
        
        %% Build Product Space
        function obj = build_productSpace(obj)
            % construct the product automaotn.
            obj = build_graph(obj);
        end
        
        function obj = build_graph(obj)
            % initialize graph
            obj.num_Z = obj.nbaParse.n_states;
            obj.num_G = obj.num_D * obj.num_Z;
            obj.GRAPH = zeros(obj.num_G,obj.num_G);
            
            % construct only feasible edges
            for nidx_D = 1:1:obj.num_D
                % neighbor indexes
                index_neighbor_D = obj.Transition{1,nidx_D};
                for nidx_Z = 1:1:obj.num_Z
                    nidx_G = obj.num_Z*(nidx_D-1) + nidx_Z;
                    for nidx_neigh_d = 1:1:size(index_neighbor_D,2)
                        nidx_next_D = index_neighbor_D(1,nidx_neigh_d);
                        
                        % 다음 D state에 해당하는 label을 구하고 이를 binary array로 변환
                        % obj.ap_ltlspec = {'a' 'b' '!a' '!b' '1'};
                        % 'a' --> [1 0 0 1 0];
                        label_next_D = obj.D{1,nidx_next_D}.label;
                        index_label_next_D = zeros(1,size(obj.ap_ltlspec,2));
                        index_label_next_D(1,obj.ap_origin_length+1:obj.ap_origin_length*2) = 1;
                        for nidx_ltlspec = 1:1:obj.ap_origin_length
                            if(label_next_D == obj.ap_ltlspec{1,nidx_ltlspec})
                                index_label_next_D(1,nidx_ltlspec) = 1;
                                index_label_next_D(1,nidx_ltlspec + obj.ap_origin_length) = 0;
                            end
                        end
                        
                        nidx_next_Z = 0;    % next automaton state
                        max_condition = 0;  % (!a) 보다는 (!a&&b)를 우대
                        is_updated = 0;
                        index_1 = 0;
                        for nidx_tranZ = 1:1:size(obj.tranZ,2)
                            tz_sel = obj.tranZ{nidx_Z,nidx_tranZ};
                            if(iscell(tz_sel))
                                % '||' 혹은 '->' 항목이 있는 경우
                                for nidx_tzs = 1:1:size(tz_sel,2)
                                    % 다음단계로 가기 위한 atomic propostion
                                    next_ap = tz_sel{1,nidx_tzs};
                                    if(isempty(next_ap))
                                        continue;
                                    end
                                    if(next_ap == size(obj.ap_ltlspec,2))
                                        index_1 = nidx_tranZ;
                                    end
                                    if(isequal(index_label_next_D(1,next_ap),ones(1,size(next_ap,2))))
                                        if(size(next_ap,2)>max_condition)
                                            is_updated = 1;
                                            nidx_next_Z = nidx_tranZ;
                                            max_condition = size(next_ap,2);
                                        end
                                    end
                                end
                            else
                                % '||' 혹은 '->' 항목이 없는 경우
                                % 다음단계로 가기 위한 atomic propostion
                                next_ap = tz_sel;
                                if(isempty(next_ap))
                                    continue;
                                end
                                if(next_ap == size(obj.ap_ltlspec,2))
                                    index_1 = nidx_tranZ;
                                end
                                if(isequal(index_label_next_D(1,next_ap),ones(1,size(next_ap,2))))
                                    if(size(next_ap,2)>max_condition)
                                        is_updated = 1;
                                        nidx_next_Z = nidx_tranZ;
                                        max_condition = size(next_ap,2);
                                    end
                                end
                            end
                        end
                        
                        % next_ap 가 '1'에 해당하는 경우 나머지를 check를 한다음에
                        % update한다.
                        if(~is_updated&&~(index_1==0))
                            nidx_next_Z = index_1;
                        end
                        
                        % 다음 state찾는데 실패... (ex_ !o 조견 만족시키지 못함)
                        %  --> 현재 이부분은 필요하지 않는다.
                        if(nidx_next_Z~=0)
                            nidx_next_G = obj.num_Z*(nidx_next_D-1) + nidx_next_Z;
                            obj.GRAPH(nidx_G,nidx_next_G) = 1;
                        end
                    end % for nidx_neigh_d = 1:1:size(index_neighbor_D,2)
                end % for nidx_Z = 1:1:obj.num_Z
            end % for nidx_D = 1:1:obj.num_D
        end
        
        %% Compute Parameters
        function obj = compute_parameters(obj,mode)
            switch mode
                case 1
                    % pre-compute w_z : [1],[2]
                    % (shortest distance from state z to the set of accepting states)
                    obj = compute_wz(obj);
                    
                case 2
                    % pre-compute w_zd : [3]
                    % (shortest distance from state (z,d) to the set of accepting states)
                    obj = compute_wzd(obj);
            end
        end
        
        function obj = compute_wz(obj)
            % compute prameter(w_z) used in referece [1],[2]
            obj.w_z = zeros(1,obj.num_Z);
            T_z = zeros(obj.num_Z,obj.num_Z);
            for nidx_tz_row = 1:1:obj.num_Z
                for nidx_tz_col = 1:1:obj.num_Z
                    T_z(nidx_tz_row,nidx_tz_col) = ~isempty(obj.tranZ{nidx_tz_row,nidx_tz_col});
                end
            end
            L_z = zeros(obj.num_Z,obj.num_Z);
            for nidx_lz_row = 1:1:obj.num_Z
                for nidx_lz_col = 1:1:obj.num_Z
                    L_z(nidx_lz_row,nidx_lz_col) = size(obj.tranZ{nidx_lz_row,nidx_lz_col},2);
                end
            end
            for nidx_tz = 1:1:obj.num_Z
                dist_tmp = zeros(1,size(obj.nbaParse.accepting_state_ids,2));
                for nidx_acc = 1:1:size(obj.nbaParse.accepting_state_ids,2)
                    [dist, ~] = dijkstra(T_z,L_z,nidx_tz);
                    dist_tmp(1,nidx_acc) = dist(1,obj.nbaParse.accepting_state_ids(1,nidx_acc));
                end
                obj.w_z(1,nidx_tz) = min(dist_tmp);
            end
        end
        
        function obj = compute_wzd(obj)
            % compute parameter(w_zd) used in referece [3]
            q_end = obj.nbaParse.accepting_state_ids;
            w_tmp = zeros(1,obj.num_G);
            T = obj.GRAPH;
            L = zeros(obj.num_G,obj.num_G);
            for nrow = 1:1:obj.num_G
                nidx_d_row = floor((nrow-1)/obj.num_Z) + 1;
                pose_row = sum(obj.D{1,nidx_d_row}.p,1)/size(obj.D{1,nidx_d_row}.p,1);
                for ncol = 1:1:obj.num_G
                    nidx_d_col  = floor((ncol-1)/obj.num_Z) + 1;
                    pose_col    = sum(obj.D{1,nidx_d_col}.p,1)/size(obj.D{1,nidx_d_col}.p,1);
                    L(nrow,ncol) = norm(pose_row - pose_col);
                end
            end
            vec_tmp = 1:1:obj.num_D;
            indexes_accept = [];
            for nidx_acc = 1:1:size(q_end,2)
                indexes_accept = [indexes_accept ...
                    (vec_tmp - 1).*obj.num_Z + q_end(1,nidx_acc)]; %#ok<AGROW>
            end
            
            for nidx_g = 1:1:obj.num_G
                [dist, ~]       = dijkstra(T,L,nidx_g);
                dist_accept     = dist(1,indexes_accept);
                [w_tmp(1,nidx_g),~] = min(dist_accept);
            end
            obj.w_zd = w_tmp;
        end
        
        function obj = compute_prePath(obj)
            % pre-compute path using Dijkstra
            obj.prePath_z = cell(obj.num_G,obj.num_Z);
            obj.prePath_g = cell(obj.num_G,obj.num_G);
            
            T = obj.GRAPH;
            L = obj.C_GRAPH;
            
            for nidx_g_from = 1:1:obj.num_G
                if obj.use_mex
                    [dist, prev] = dijkstra_mex(T,L,nidx_g_from);
                else
                    [dist, prev] = dijkstra(T,L,nidx_g_from);
                end
                for nidx_z_to = 1:1:obj.num_Z
                    pre_path_tmp = cell(1,obj.num_D);
                    pre_cost_tmp = inf*ones(1,obj.num_D);
                    for nidx_d_to = 1:1:obj.num_D
                        idx_g_to = (nidx_d_to - 1)*obj.num_Z + nidx_z_to;
                        
                        dist_accept = dist(1,idx_g_to);
                        [min_dist,index_min_dist] = min(dist_accept);
                        if(~isequal(min_dist,inf))
                            index_end_point = idx_g_to(1,index_min_dist);
                            [highlevel_plan]  = findPath(obj,index_end_point,prev);
                            obj.prePath_g{nidx_g_from,idx_g_to} = highlevel_plan;
                            
                            pre_path_tmp{1,nidx_d_to} = highlevel_plan;
                            pre_cost_tmp(1,nidx_d_to) = min_dist;
                        end
                    end
                    
                    if(nidx_z_to==2)
                    end
                    
                    [~,idx_min] = min(pre_cost_tmp);
                    obj.prePath_z{nidx_g_from,nidx_z_to} = pre_path_tmp{1,idx_min};
                end
            end
        end
        
        %% Find High-Level Plan
        function [highlevel_plan,is_success] = find_highLevelPlan(obj,d_init,q_init,q_end,mode_L,w)
            start_index = (d_init-1)*obj.num_Z + q_init;
            T = obj.GRAPH;
            L = zeros(obj.num_G,obj.num_G);
            switch mode_L
                case 'weight'
                    for nrow = 1:1:obj.num_G
                        idx_d_row = floor((nrow-1)/obj.num_Z) + 1;
                        for ncol = 1:1:obj.num_G
                            idx_d_col = floor((ncol-1)/obj.num_Z) + 1;
                            L(nrow,ncol) = w(idx_d_row,idx_d_col);
                        end
                    end
                case 'cost'
                   L = obj.C_GRAPH;
            end
            
            is_success = 0;
            if(strcmp(mode_L,'random'))
                % find a random path (do not consider a cost of path)
                [idx_discovered, prev] = findRandomPath(T,start_index);
                
                % find minimum dist (or cost) path to accepting states
                vec_tmp = 1:1:obj.num_D;
                indexes_accept = [];
                for nidx_acc = 1:1:size(q_end,2)
                    indexes_accept = [indexes_accept ...
                        (vec_tmp - 1).*obj.num_Z + q_end(1,nidx_acc)]; %#ok<AGROW>
                end
                
                idx_valid = find(idx_discovered == 1);
                idx_valid_accept = intersect(idx_valid,indexes_accept);
                if(~isempty(idx_valid_accept))
                    is_success = 1;
                    index_end_point = idx_valid_accept(1,randi(size(idx_valid_accept,2),1));
                end
                
            else
                % apply dijkstra's algorithm
                if obj.use_mex
                    [dist, prev] = dijkstra_mex(T,L,start_index);
                else
                    [dist, prev] = dijkstra(T,L,start_index);
                end
                
                % Find minimum dist (or cost) path to accepting states
                vec_tmp = 1:1:obj.num_D;
                indexes_accept = zeros(1,obj.num_D*size(q_end,2));
                for nidx_acc = 1:1:size(q_end,2)
                    idx2update      = ((nidx_acc-1)*obj.num_D+1):1:(nidx_acc*obj.num_D);
                    indexes_accept(1,idx2update) = (vec_tmp - 1).*obj.num_Z + q_end(1,nidx_acc);
                end
                
                dist_accept = dist(1,indexes_accept);
                min_dist_accept = min(dist_accept);
                [~,index_min_dist] = find(dist_accept == min_dist_accept);
                len_index_min_dist = size(index_min_dist,2);
                if(len_index_min_dist>1)
                    index_min_dist = index_min_dist(1,randi(len_index_min_dist,1));
                end
                
                if(~isequal(min_dist_accept,inf))
                    is_success = 1;
                    index_end_point = indexes_accept(1,index_min_dist);
                end
            end
            
            if(is_success)
                highlevel_plan = findPath(obj,index_end_point,prev);
            else
                is_success = 0;
                highlevel_plan = [];
            end
        end
        
        function highlevel_plan = findPath(obj,index_endpoint,node_parent)
            % index_endpoint : index of endpoint state
            % node_parent : array of index of parent nodes
            
            HP = zeros(1,1000);
            ind_HP = 0;
            
            ind_HP = ind_HP + 1;
            HP(1,ind_HP) = floor((index_endpoint-1)/obj.num_Z) + 1;
            index_parent = index_endpoint;
            
            while(1)
                index_parent = node_parent(1,index_parent);
                if(index_parent == 0)
                    break;
                end
                
                ind_HP = ind_HP + 1;
                HP(1,ind_HP) = floor((index_parent-1)/obj.num_Z) + 1;
                
            end
            HP = HP(1,1:ind_HP);
            highlevel_plan = fliplr(HP);
        end
        
        %% Select Initial High-Level State
        function [z_from, d_from, z_to] = find_discreteTarget(obj,T)
            % used in reference [2]
            
            is_procedure_end = 0;
            % select z_from
            z_avail = T.v.alpha(1,1:1:T.num_node);
            z_avail = unique(z_avail','rows')';
            
            prob = zeros(1,size(z_avail,2));
            for nidx_z = 1:1:size(z_avail,2)
                prob(1,nidx_z) = 2^(-1*obj.w_z(nidx_z));
            end
            prob = prob/sum(prob,2);
            cnt_iter = 0;
            while(~is_procedure_end)
                z_from = z_avail(find(rand<cumsum(prob),1,'first'));
                
                % select d_from
                [~,idx_candidate] = find(z_avail == z_from);
                d_candidate = T.v.d(1,idx_candidate);
                d_candidate = unique(d_candidate','rows')';
                d_from      = d_candidate(1,randi(size(d_candidate,2),1));
                
                % select z_to
                if(isequal(z_avail,1:obj.num_Z))
                    z_to_candidate = z_avail;
                else
                    z_to_candidate = setxor(1:obj.num_Z,z_avail);
                end
                
                idx_nonempty=~cellfun(@isempty,obj.tranZ(z_from,:));
                [~,idx_z_found] = find(idx_nonempty==1);
                
                [idx_z_common,~,~] = intersect(idx_z_found,z_to_candidate);
                if(~isempty(idx_z_common))
                    index_selected = randi(size(idx_z_common,2),1);
                    z_to = idx_z_common(1,index_selected);
                    is_procedure_end = 1;
                end
                cnt_iter = cnt_iter + 1;
                if(cnt_iter>=100)
                    error('failed in finding discrete target\n');
                end
            end
        end
        
        function z_to = find_targetPoint(obj,z_from)
            % used in referece [3]
            
            z_found     = zeros(1,obj.num_Z);
            idx_z       = 0;
            for nidx_npz = 1:1:obj.num_Z
                tran_z_sel = obj.tranZ{z_from,nidx_npz};
                if(isempty(tran_z_sel))
                    continue;
                end
                if(nidx_npz == z_from)
                    continue;
                end
                if(size(tran_z_sel,2)==1)
                    idx_z = idx_z + 1;
                    z_found(idx_z) = nidx_npz;
                end
            end
            z_found =  z_found(1:idx_z);
            
            if(size(z_found,2) == 1)
                z_to        = z_found;
            else
                z_to        = z_found(1,randi(size(z_found,2)));
            end
        end
        
        % proposed version
        function z_to = find_targetPoint_proposed(obj,T,z_from,is_uniform)
            z_avail = T.v.alpha(1,1:T.num_node);
            
            z_found     = zeros(1,obj.num_Z);
            idx_z       = 0;
            for nidx_npz = 1:1:obj.num_Z
                tran_z_sel = obj.tranZ{z_from,nidx_npz};
                if(isempty(tran_z_sel))
                    continue;
                end
                if(nidx_npz == z_from)
                    continue;
                end
                if(size(tran_z_sel,2)==1)
                    idx_z = idx_z + 1;
                    z_found(idx_z) = nidx_npz;
                end
            end
            z_found =  z_found(1:idx_z);
            
            if(size(z_found,2) == 1)
                z_to = z_found;
            else
                num_n_per_z = zeros(1,size(z_found,2));
                
                for nidx_npz = 1:1:size(z_found,2)
                    [~,idx_found] = find(z_avail == z_found(1,nidx_npz));
                    num_n_per_z(1,nidx_npz) = length(idx_found);
                end
                
                if is_uniform
                    prob2sel = ones(1,size(num_n_per_z,2))/size(num_n_per_z,2);
                else
                    if(sum(num_n_per_z,2) == 0)
                        prob2sel = ones(1,size(num_n_per_z,2))/size(num_n_per_z,2);
                    else
                        prob2sel = ones(1,size(num_n_per_z,2)) - num_n_per_z/sum(num_n_per_z,2);
                    end
                end
                
                z_to = z_found(find(rand<cumsum(prob2sel),1,'first'));
            end
        end
        
        function [z_to,d_to,g_to] = find_targetPoint_proposed_old(obj,T,z_from,d_from)
            g_from = (d_from-1)*obj.num_Z + z_from;
            
            z_avail = T.v.alpha(1,1:T.num_node);
            d_avail = T.v.d(1,1:T.num_node);
            
            idx_nonempty_z  = ~cellfun(@isempty,obj.tranZ(z_from,:));
            [~,z_found] = find(idx_nonempty_z == 1);
            num_n_per_z     = zeros(1,size(z_found,2));
            
            for nidx_npz = 1:1:size(z_found,2)
                [~,idx_found] = find(z_avail == z_found(1,nidx_npz));
                num_n_per_z(1,nidx_npz) = length(idx_found);
            end
            
            prob2sel_z  = ones(1,size(num_n_per_z,2)) - num_n_per_z/sum(num_n_per_z,2);
            z_to        = z_found(find(rand<cumsum(prob2sel_z),1,'first'));
            
            d_to_possible   = 1:1:obj.num_D;
            g_to_possible   = obj.num_Z*(d_to_possible - ones(1,obj.num_D)) + z_to*ones(1,obj.num_D);
            
            idx_nonempty_g  = ~cellfun(@isempty,obj.prePath_g(g_from,g_to_possible));
            [~,idx_g_found] = find(idx_nonempty_g == 1);
            d_found         = d_to_possible(1,idx_g_found);
            g_found         = g_to_possible(1,idx_g_found);
            num_n_per_g     = zeros(1,size(idx_g_found,2));
            for nidx_d = 1:1:size(d_found,2)
                [~,idx_found]           = find(d_avail == d_found(1,nidx_d));
                num_n_per_g(1,nidx_d)   = length(idx_found);
            end
            prob2sel_g    = ones(1,size(num_n_per_g,2)) - num_n_per_g/sum(num_n_per_g,2);
            d_to          = d_found(find(rand<cumsum(prob2sel_g),1,'first'));
            g_to          = g_found(find(rand<cumsum(prob2sel_g),1,'first'));
        end
        
        %% Plot
        function Plot(obj,figurenum)
            xmax = max(obj.points(:,1));
            ymax = max(obj.points(:,2));
            xmin = min(obj.points(:,1));
            ymin = min(obj.points(:,2));
            
            figure(figurenum); hold on; axis equal; axis tight;
            for nf = 1:1:size(obj.D,2)
                x = obj.D{nf}.p(:,1);
                y = obj.D{nf}.p(:,2);
                if(nf>=obj.index_RI)
                    hp = fill(x,y,[0.7608 0.7608 0.7608]);
                    set(hp,'EdgeColor','k','linewidth',4);
                    htext = sprintf('%s=%d',obj.D{nf}.label,nf);
                    text(mean(x),mean(y),htext);
                else
                    hcolor = (ones(1,3)*70 +randi((255-70),1,3))/255;
                    hp = fill(x,y,hcolor);
                    set(hp,'EdgeColor',hcolor);
                    htext = sprintf('%d',nf);
                    text(mean(x),mean(y),htext);
                end
            end
            
            % plot obstacles
            for no = 1:1:size(obj.Obs,2)
                x = obj.Obs{no}.p(:,1);
                y = obj.Obs{no}.p(:,2);
                hp = fill(x,y,[55 48 45]/255);
                set(hp,'EdgeColor','k','linewidth',4);
            end
            
            t=title('Discrete Abstraction');
            set(t,'fontsize',12);
            xlim([xmin xmax]);
            ylim([ymin ymax]);
        end
        
        function Plot_ver2(obj,figurenum)
            xmax = max(obj.points(:,1));
            ymax = max(obj.points(:,2));
            xmin = min(obj.points(:,1));
            ymin = min(obj.points(:,2));
            
            plot_size = [500 500]; % width, height

            hFig = figure(figurenum); clf; hold on; axis equal; axis tight; box on;
            set(hFig,'Position',[50 50 plot_size(1,1) plot_size(1,2)]);
            for nf = 1:1:size(obj.D,2)
                x = obj.D{nf}.p(:,1);
                y = obj.D{nf}.p(:,2);
                if(nf>=obj.index_RI)
                    hp = fill(x,y,rgb('GreenYellow'));
                    set(hp,'EdgeColor','k','linewidth',2,'FaceAlpha',0.5);
                    text2print = sprintf('%s',obj.D{nf}.label);
                    htxt = text(mean(x),mean(y),text2print,'HorizontalAlignment','center');
                    set(htxt,'fontweight','normal','fontsize',20,'Color',rgb('Black'));
                else
                    x_ = [x; x(1)];
                    y_ = [y; y(1)];
                    plot(x_,y_,':','LineWidth',1,'Color',rgb('Gray'));
                end
            end
            
            % plot obstacles
            for no = 1:1:size(obj.Obs,2)
                x = obj.Obs{no}.p(:,1);
                y = obj.Obs{no}.p(:,2);
                hp = fill(x,y,[55 48 45]/255);
                set(hp,'EdgeColor','k','linewidth',1);
            end
            
            xlim([xmin xmax]);
            ylim([ymin ymax]);
        end
    end
end
