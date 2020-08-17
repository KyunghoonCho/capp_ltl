% Check automaton state.
function [is_fail,nidx_Z] = checkAutomaton(traj,z_init,DP)
is_fail = 0;

nidx_Z = z_init;
trace = zeros(1,size(traj,2));
trace(1,1) = nidx_Z;

for nidx_t = 2:1:size(traj,2)
    x_next = traj(:,nidx_t);
    
    % 다음 D state에 해당하는 label을 구하고 이를 binary array로 변환
    % obj.ap_ltlspec = {'a' 'b' '!a' '!b' '1'};
    % 'a' --> [1 0 0 1 0];
    label_next_D = findLabel(DP,x_next);
    index_label_next_D = zeros(1,size(DP.ap_ltlspec,2));
    index_label_next_D(1,DP.ap_origin_length+1:DP.ap_origin_length*2) = 1;
    for nidx_ltlspec = 1:1:DP.ap_origin_length
        if(label_next_D == DP.ap_ltlspec{1,nidx_ltlspec})
            index_label_next_D(1,nidx_ltlspec) = 1;
            index_label_next_D(1,nidx_ltlspec + DP.ap_origin_length) = 0;
        end
    end
    
    nidx_next_Z = 0;    % next automaton state
    max_condition = 0;  % (!a) 보다는 (!a&&b)를 우대
    is_updated = 0;
    index_1 = 0;
    for nidx_tranZ = 1:1:size(DP.tranZ,2)
        tz_sel = DP.tranZ{nidx_Z,nidx_tranZ};
        if(iscell(tz_sel))
            % '||' 혹은 '->' 항목이 있는 경우
            for nidx_tzs = 1:1:size(tz_sel,2)
                % 다음단계로 가기 위한 atomic propostion
                next_ap = tz_sel{1,nidx_tzs};
                if(isempty(next_ap))
                    continue;
                end
                if(next_ap == size(DP.ap_ltlspec,2))
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
            if(next_ap == size(DP.ap_ltlspec,2))
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
    
    if(nidx_next_Z == 0)
        is_fail = 1;
        break;
    end
    nidx_Z = nidx_next_Z;
    trace(1,nidx_t) = nidx_Z;
end
end
