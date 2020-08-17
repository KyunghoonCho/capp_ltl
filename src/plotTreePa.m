% Plot tree per automaton.
function hFig = plotTreePa(DP,T,idx_ws,figurenum,map,cost_type,automaton_state,is_black)

if is_black
    bcolor = 'w';
else
    bcolor = 'k';
end

map_xlen = map.xmax - map.xmin;
map_ylen = map.ymax - map.ymin;
width = map_xlen * 0.6;
height = map_ylen * 0.6;

hFig = figure(figurenum); clf; hold on; axis equal; axis tight;
set(hFig,'PaperUnits', 'inches', 'PaperPosition',[0 0 width height]);

if(strcmp(cost_type,'costmap'))
    C_plot = map.C;
    [~,h] = contourf(map.X,map.Y,C_plot,100);
    set(h,'LineColor','none');
end

%% plot background
for nf = 1:1:size(DP.D,2)
    x = DP.D{nf}.p(:,1)';
    y = DP.D{nf}.p(:,2)';
    points2plot = [x x(1); y y(1)];
    
    if(nf>=DP.index_RI)
        hp = fill(x,y,rgb('BlueViolet'));
        set(hp,'EdgeColor',bcolor,'linewidth',1,'FaceAlpha',0.7);
        text2print = sprintf('%s',DP.D{nf}.label);
        htxt = text(mean(x),mean(y),text2print,'HorizontalAlignment','center');
        set(htxt,'fontweight','bold','fontsize',20,'Color',rgb('Azure'));
    else
        plot(points2plot(1,:),points2plot(2,:),'-','color',rgb('DimGray'),'linewidth',0.5);
    end
end

%% plot obstacles
for no = 1:1:size(DP.Obs,2)
    x = DP.Obs{no}.p(:,1)';
    y = DP.Obs{no}.p(:,2)';
    hp = fill(x,y,rgb('Gray'));
    set(hp,'EdgeColor',bcolor,'linewidth',2);
end

%% plot edge
for nidx_t = 1:1:T.num_node    
    if(nidx_t>1)
        if(T.v.alpha(1,nidx_t)==automaton_state)
            idx_parent = T.e.parent(1,nidx_t);
            % pseg = [T.v.x(1:2,idx_parent) T.v.x(1:2,nidx_t)];
            pseg = T.v.pseg{nidx_t}(idx_ws,:);
            
            if(T.e.rewire(1,nidx_t) == 1)
                plot(pseg(1,:),pseg(2,:),'-','Color',rgb('Crimson'),'linewidth',0.75);
            else
                plot(pseg(1,:),pseg(2,:),'-','Color',rgb('LightSkyBlue'),'linewidth',0.75);
            end
            % plot(pseg(1,:),pseg(2,:),'-','Color',rgb('YellowGreen'),'linewidth',0.5);
        end
    end
end

%% plot vertex
for nidx_t = 1:1:T.num_node
    if(T.v.alpha(1,nidx_t)==automaton_state)
        x_plot = T.v.x(1:2,nidx_t);
        plot(x_plot(1),x_plot(2),'o','MarkerSize',1,'linewidth',0.3,...
            'MarkerFaceColor',rgb('OliveDrab'),'MarkerEdgeColor',rgb('OliveDrab'));
    end
end

xlim([map.xmin map.xmax]);
ylim([map.ymin map.ymax]);
set(gca,'layer','top');
set(gca,'LooseInset',get(gca,'TightInset'));

% black
if is_black; set(hFig,'Color','k'); end
set(gcf, 'InvertHardCopy','off');
end
