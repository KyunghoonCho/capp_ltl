% Plot tree.
function hFig = plotTree(DP,T,figurenum,idx_ws,zinit,dinit,x0,map,cost_type,is_black)

if is_black
    bcolor = 'w';
else
    bcolor = 'k';
end

map_xlen = map.xmax - map.xmin;
map_ylen = map.ymax - map.ymin;
width = map_xlen * 0.4;
height = map_ylen * 0.4;

hFig = figure(figurenum); clf; hold on; axis equal; axis tight; box on;
set(hFig,'PaperUnits', 'inches', 'PaperPosition',[0 0 width height]);

if(strcmp(cost_type,'costmap'))
    [~,h] = contourf(map.X,map.Y,map.C,50);
    set(h,'LineColor','none');
end

%% plot obstacles
for no = 1:1:size(DP.Obs,2)
    x = DP.Obs{no}.p(:,1)';
    y = DP.Obs{no}.p(:,2)';
    hp = fill(x,y,[62 62 62]/255);
    set(hp,'EdgeColor',bcolor,'linewidth',1,'FaceAlpha',0.8);
end

%% plot edge
for nidx_t = 1:1:T.num_node
    if(nidx_t>1)
        % idx_parent = T.e.parent(1,nidx_t);
        % pseg_temp = [T.v.x(idx_ws,idx_parent) T.v.x(idx_ws,nidx_t)];
        pseg_ws = T.v.pseg{nidx_t}(idx_ws,:);
        
        if(T.e.rewire(1,nidx_t) == 1)
            plot(pseg_ws(1,:),pseg_ws(2,:),'-','Color',rgb('Violet'),'linewidth',0.5);
        else
            plot(pseg_ws(1,:),pseg_ws(2,:),'-','Color',rgb('LightSkyBlue'),'linewidth',0.5);
        end
    end
end

%% plot vertex
for nidx_t = 1:1:T.num_node
    x_v_ws = T.v.x(idx_ws,nidx_t);
    plot(x_v_ws(1),x_v_ws(2),'o','MarkerSize',0.5,'linewidth',0.5,...
        'MarkerFaceColor',rgb('OliveDrab'),'MarkerEdgeColor',rgb('LightCyan'));
end

x_init = T.v.x(1:2,1);
plot(x_init(1),x_init(2),'s','linewidth',4,'color',rgb('Green'),'MarkerSize',15);

%% plot background
for nf = 1:1:size(DP.D,2)
    x = DP.D{nf}.p(:,1)';
    y = DP.D{nf}.p(:,2)';
    
    if(nf>=DP.index_RI)
        hp = fill(x,y,rgb('BlueViolet'));
        set(hp,'EdgeColor',bcolor,'linewidth',1,'FaceAlpha',0.8);
        text2print = sprintf('%s',DP.D{nf}.label);
        htxt = text(mean(x),mean(y),text2print,'HorizontalAlignment','center');
        set(htxt,'fontweight','bold','fontsize',20,'Color',rgb('Azure'));
    else
        points2plot = [x x(1); y y(1)];
        plot(points2plot(1,:),points2plot(2,:),'-','color',rgb('DimGray'),'linewidth',0.2);
    end
end

%% plot start cell
points2plot = [DP.D{dinit}.p(:,1)' DP.D{dinit}.p(1,1)'; ...
    DP.D{dinit}.p(:,2)' DP.D{dinit}.p(1,2)'];
plot(points2plot(1,:),points2plot(2,:),'--','color',rgb('Orange'),'linewidth',1.2);
if(dinit>=DP.index_RI)
    text2print = sprintf('%s',DP.D{dinit}.label);
    htxt = text(mean(DP.D{dinit}.p(:,1)),mean(DP.D{dinit}.p(:,2)),text2print);
    set(htxt,'fontsize',12,'fontweight','bold','color',bcolor);
end

% x = traj_seg(1,:);
% y = traj_seg(2,:);
% z = zeros(size(traj_seg(1,:)));
% S = surface([x;x],[y;y],[z;z],...
%     'facecol','no',...
%     'edgecol','interp',...
%     'linewidth',4,...
%     'edgealpha',.3,...
%     'edgecolor',rgb('OrangeRed'));

x0_ws   = x0(idx_ws,:);
txt_x0  = sprintf('(%d,%d)',zinit,dinit);
htext   = text(x0_ws(1),x0_ws(2)-(map.ymax-map.ymin)/20,txt_x0);
set(htext,'fontsize',12,'fontweight','bold','color',bcolor);
plot(x0_ws(1),x0_ws(2),'o','MarkerSize',10,'linewidth',3,'color',rgb('DarkOrange'));

xlim([map.xmin map.xmax]);
ylim([map.ymin map.ymax]);
set(gca,'layer','top');
set(gca,'LooseInset',get(gca,'TightInset'));

% black
if is_black; set(hFig,'Color','k'); end
set(gcf, 'InvertHardCopy','off');
end
