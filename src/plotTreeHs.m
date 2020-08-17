% Plot tree with highlevel-plan.
function hFig = plotTreeHs(DP,T,figurenum,idx_ws,zinit,dinit,x0,map,cost_type,highlevel_plan,g_target,is_black)

[LIA,~] = ismember(dinit,highlevel_plan);
if ~LIA
    highlevel_plan = [dinit, highlevel_plan];
end

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

if(strcmp(cost_type,'costmap'))
    C_plot = map.C;
    [~,h] = contourf(map.X,map.Y,C_plot,100);
    set(h,'LineColor','none');
end
c = colorbar;
c.Color = bcolor;

%% plot obstacle
for no = 1:1:size(DP.Obs,2)
    x = DP.Obs{no}.p(:,1)';
    y = DP.Obs{no}.p(:,2)';
    hp = fill(x,y,rgb('Gray'));
    set(hp,'EdgeColor',bcolor,'linewidth',1,'FaceAlpha',0.7);
end

%% plot edge
for nidx_t = 1:1:T.num_node
    if(nidx_t>1)
        % idx_parent = T.e.parent(1,nidx_t);
        % pseg_temp = [T.v.x(idx_ws,idx_parent) T.v.x(idx_ws,nidx_t)];
        pseg_ws = T.v.pseg{nidx_t}(idx_ws,:);
        
        if(T.e.rewire(1,nidx_t) == 1)
            plot(pseg_ws(1,:),pseg_ws(2,:),'-','Color',rgb('Crimson'),'LineWidth',0.75);
        else
            plot(pseg_ws(1,:),pseg_ws(2,:),'-','Color',rgb('DeepSkyBlue'),'LineWidth',0.75);
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
plot(x_init(1),x_init(2),'s','linewidth',2,'color',rgb('Green'),'MarkerSize',10);

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

%% plot high-level plan
for nf = 1:1:size(DP.D,2)
    x = DP.D{nf}.p(:,1)';
    y = DP.D{nf}.p(:,2)';
    points2plot = [x x(1); y y(1)];
    [LIA,LOCB] = ismember(nf,highlevel_plan);
    if(LIA)
        hp = fill(x,y,rgb('Salmon'));
        set(hp,'EdgeColor',bcolor,'linewidth',1,'FaceAlpha',0.7);
               
%         switch LOCB
%             case 1
%                 plot(points2plot(1,:),points2plot(2,:),'-','color',rgb('Orange'),'linewidth',2.5);
%             case size(highlevel_plan,2)
%                 % plot(points2plot(1,:),points2plot(2,:),'-','color',rgb('DodgerBlue'),'linewidth',2.5);
%         end
    else
       % plot(points2plot(1,:),points2plot(2,:),'-','color',rgb('DimGray'),'linewidth',0.5);
    end
    
    if nf == dinit
        plot(points2plot(1,:),points2plot(2,:),'-','color',rgb('Orange'),'linewidth',2.5);
    end
    
    if(nf == g_target)
        plot(points2plot(1,:),points2plot(2,:),'-','color',rgb('Red'),'linewidth',2.5);
    end
end

x0_ws   = x0(idx_ws,:);
txt_x0  = sprintf('<%d,%d>',zinit,dinit);
htext   = text(x0_ws(1)+(map.xmax-map.xmin)/80,x0_ws(2)-(map.ymax-map.ymin)/60,txt_x0);
set(htext,'fontsize',11,'fontweight','bold','color',bcolor);
plot(x0_ws(1),x0_ws(2),'o','MarkerSize',5,'linewidth',1,'color',rgb('DarkOrange'));

xlim([map.xmin map.xmax]);
ylim([map.ymin map.ymax]);
set(gca,'layer','top');
set(gca,'LooseInset',get(gca,'TightInset'));
axis off;

% black
if is_black; set(hFig,'Color','k'); end
set(gcf, 'InvertHardCopy','off');
end