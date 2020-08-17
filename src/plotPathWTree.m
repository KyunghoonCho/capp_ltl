% Plot path with tree.
function hFig = plotPathWTree(DP,T,idx_ws,map,path,cost_path,time_path,figurenum,cost_type,is_black)

if is_black
    bcolor = 'w';
else
    bcolor = 'k';
end

map_xlen = map.xmax - map.xmin;
map_ylen = map.ymax - map.ymin;
width = map_xlen * 0.6;
height = map_ylen * 0.6;

hFig = figure(figurenum); clf; hold on; axis equal; axis tight; box on;
set(hFig,'PaperUnits', 'inches', 'PaperPosition',[0 0 width height]);

% txt_title = sprintf('cost: %.2f, time: %.2f',cost_path,time_path);
% htitle = title(txt_title);
% set(htitle,'fontsize',15);

if(strcmp(cost_type,'costmap'))
    C_plot = map.C;
    [~,h] = contourf(map.X,map.Y,C_plot,100);
    set(h,'LineColor','none');
end
c = colorbar;
c.Color = bcolor;

%% plot obstacle
for nidx_o = 1:1:size(DP.Obs,2)
    x = DP.Obs{nidx_o}.p(:,1)';
    y = DP.Obs{nidx_o}.p(:,2)';
    hp = fill(x,y,rgb('Gray'));
    set(hp,'EdgeColor',bcolor,'linewidth',1,'FaceAlpha',0.7);
end

%% plot edge
for nidx_t = 1:1:T.num_node
    if(nidx_t>1)
        pseg_ws = T.v.pseg{nidx_t}(idx_ws,:);
        
        if(T.e.rewire(1,nidx_t) == 1)
            % plot(pseg_ws(1,:),pseg_ws(2,:),'-','Color',rgb('Violet'),'linewidth',0.5);
            plot(pseg_ws(1,:),pseg_ws(2,:),'-','Color',rgb('LightSkyBlue'),'linewidth',0.5);
        else
            plot(pseg_ws(1,:),pseg_ws(2,:),'-','Color',rgb('LightSkyBlue'),'linewidth',0.5);
        end
    end
end

%% plot vertex
for nidx_t = 1:1:T.num_node
    x_ws = T.v.x(idx_ws,nidx_t);
    plot(x_ws(1),x_ws(2),'o','MarkerSize',0.5,'linewidth',0.5,...
        'MarkerFaceColor',rgb('OliveDrab'),'MarkerEdgeColor',rgb('LightCyan'));
end

x_init = T.v.x(idx_ws,1);
plot(x_init(1),x_init(2),'s','linewidth',4,'color',rgb('Green'),'MarkerSize',15);

%% plot path
if(~isempty(path))
    path_ws = path(idx_ws,:);
    plot(path_ws(1,:),path_ws(2,:),'-','linewidth',3,'color',rgb('Yellow'));
    plot(path_ws(1,:),path_ws(2,:),'-','linewidth',2.25,'color',rgb('OrangeRed'));
end

%% plot background
for nf = 1:1:size(DP.D,2)
    x = DP.D{nf}.p(:,1)';
    y = DP.D{nf}.p(:,2)';
    
    if(nf>=DP.index_RI)
        hp = fill(x,y,rgb('BlueViolet'));
        set(hp,'EdgeColor',bcolor,'linewidth',1,'FaceAlpha',0.7);
        text2print = sprintf('%s',DP.D{nf}.label);
        htxt = text(mean(x),mean(y),text2print,'HorizontalAlignment','center');
        set(htxt,'fontweight','bold','fontsize',20,'Color',rgb('Azure'));
    else
        points2plot = [x x(1); y y(1)];
        plot(points2plot(1,:),points2plot(2,:),'-','color',rgb('DimGray'),'linewidth',0.2);
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

