% Load a Buchi Automaton from a Never Claim.
%
%usage
%-----
%   formula = '[]<>p'
%   path_to_ltl2ba = '/bin/ltl2ba'
%   ba = parse_never_claim(formula, path_to_ltl2ba)
%
%input
%-----
%   formula = string of LTL formula for input to ltl2ba
%   path_to_ltl2ba = optional, if absent then default paths used
%
%output
%------
%   ba = Buchi Automaton struct with fields:
%       ba.states = state names, order matters, it's used by adj indices
%       ba.n_states = total # of states
%       ba.initial_states = {} of initial state names
%       ba.initial_state_ids = indices of states in ba.initial_states wrt
%           ba.states
%       ba.accepting_states = {} of accepting state names
%       ba.accepting_state_ids = indices of states in ba.accepting_states
%       ba.adj = {} containing {} of str, each str is a conjunct from the
%           negation normal form
%              = 'prop' | '!prop' | '1'
%
%note
%----
%   ltl2ba may be on the path, but if that path has been exported from your
%   .bash_profile, it's still not found, because MATLAB does not source it
%
%   strsplit is for R2012a, has changed order of arguments in R2013a
%   and again changed for R2014a
%
%depends
%-------
%   ltl2ba: http://www.lsv.ens-cachan.fr/~gastin/ltl2ba/
%   strsplit (Matlab 2013a function)
%
%about
%-----
%   2013.08.16 Ioannis Filippidis, jfilippidis@gmail.com
%   see also plot_buchi_automaton
%   2013.08.17 Modified by Eric Wolff, ewolff@caltech.edu
%       - bug fix: disjunctions in guards
%       - bug fix: never claims with 'accept_all: skip' states
%   2014.08.25 Modified by Kyunghoon Cho, whynot0923@gmail.com
%       - bug fix: change order in 'strsplit';

function [ba] = parseNeverClaim(formula, path_to_ltl2ba)

%% arg
if nargin < 1
    formula = '[]<>(!p1 && p2)';
    warning('ltl2ba:formula', 'No formula passed.')
end

if isempty(formula)
    warning('ltl2ba:formula', 'Empty formula')
end

if ~ischar(formula)
    error('formula must be a string.')
end

% assume path_to_ltl2ba on 
if nargin < 2
    if ismac
        path_to_ltl2ba = '/Users/ltl2ba';
    elseif ispc
        path_to_ltl2ba = 'C:\ltl2ba.exe';
    else
        error('Not mac, not pc, so path to ltl2ba unknown')
    end
end

%% call ltl2ba
[s, ltl2ba_output] = system([path_to_ltl2ba, ' -f "', formula, '"'] );

% disp('ltl2ba output:')
disp(ltl2ba_output)

if s ~= 0
    error('Error when converting LTL to Buchi.')
end

%% parse ltl2ba output
ltl2ba_output = textscan(ltl2ba_output, '%s', 'delimiter', '\n');
ltl2ba_output = ltl2ba_output{1};

states = {};
for i=1:size(ltl2ba_output, 1)
    line = ltl2ba_output{i, 1};
    
    if ~isempty(strfind(line, 'never') ) || ~isempty(strfind(line, '}') )
        continue
    end
    
    s = line(1:2);
    if ~strcmp(s, 'if') && ~strcmp(s, 'fi') && ~strcmp(s, '::') && ~strcmp(s,'sk')  % sk == skip
        states = [states, {line(1:end-1) } ];
    end
end

within_state = 0;
adj = cell(length(states) );
for i=1:size(ltl2ba_output, 1)
    line = ltl2ba_output{i, 1};
    
    if ~isempty(strfind(line, 'never') ) || ~isempty(strfind(line, '}') )
        continue
    end
    
    s = line(1:2);
    if strcmp(s, 'if')
        within_state = 1;
    end
    
    if strcmp(s, 'fi')
        within_state = 0;
    end
    
    if strcmp(s,'sk')   % sk == skip        
        from_state = cur_state;
        [~, from_state_id] = find_states(states, from_state);
        to_state_id = from_state_id;

        % parse guard
        adj{from_state_id, to_state_id} = {};
        props = {'1'};
        adj{from_state_id, to_state_id}{1} = props;
%         fprintf('Found transition: %s ---> %s with guard(s):\n',from_state,to_state);
%         disp(props);

    end
    
    if strcmp(s, '::')
        assert( within_state == 1 );
        
        from_state = cur_state;
        [~, from_state_id] = find_states(states, from_state);

        guards_goto = line(3:end);
        guards_goto = strsplit(guards_goto,'->');
        guards = guards_goto{1};
        goto = guards_goto{2};

        % parse to_state
        goto = strsplit(goto, ' ');
        to_state = goto(end);
        to_state = to_state{1};
        [~, to_state_id] = find_states(states, to_state);

        % parse guards
%         fprintf('Found transition: %s ---> %s, with guard(s):\n',from_state,to_state);
        adj{from_state_id, to_state_id} = {};
        guards = strsplit(guards,'||');
        for j=1:length(guards)
            guard = guards{j};
            guard = strrep(guard, '(', '');
            guard = strrep(guard, ')', '');
            props = strsplit(guard,'&&');
            props = cellfun(@strtrim, props, 'UniformOutput', false);       
%             disp(props);
            adj{from_state_id, to_state_id}{j} = props;
        end
    end
    
    if ~strcmp(s, 'if') && ~strcmp(s, 'fi') && ~strcmp(s, '::') && ~strcmp(s,'sk')
        cur_state = line(1:end-1);
        %cur_state_id = find(strcmp(cur_state, states) );
    end
end

ba.states = states; % names (order same as indices in adj matrix)
ba.n_states = length(states);

[init_states, init_state_ids] = find_states(states, 'init');
ba.initial_state_ids = init_state_ids;
ba.initial_states = init_states;

[accepting_states, accepting_state_ids] = find_states(states, 'accept');
ba.accepting_state_ids = accepting_state_ids;
ba.accepting_states = accepting_states;

ba.adj = adj;

return

function [desired_states, desired_state_ids] = find_states(states, s)
% filter states by  string
which_ = cellfun(@(x) ~isempty(x), strfind(states, s) );
desired_state_ids = find(which_);
desired_states = states(desired_state_ids);
