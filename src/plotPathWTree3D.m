% Plot path with tree (3D).
function hFig = plotPathWTree3D(DP,T,idx_ws,map,path,automaton_path,d_sel,z_sel,v_target,...
    cost_path,time_path,figurenum,cost_type,highlevel_plan,g_target,is_long,traj_seg,is_black)

[LIA,~] = ismember(d_sel,highlevel_plan);
if ~LIA
    highlevel_plan = [d_sel, highlevel_plan];
end

if is_black
    bcolor = 'w';
    mcolor = rgb('LightGray');
    obs_color = rgb('LightGray');
else
    bcolor = 'k';
    mcolor = rgb('Silver');
    obs_color = rgb('Gray');
end

height = 1.2 * DP.num_Z * 2;
hFig = figure(figurenum); clf; hold on; axis equal; axis tight; box on;
set(hFig,'PaperUnits', 'inches', 'PaperPosition',[0 0 2.5*2 height]);

% txt_title = sprintf('cost: %.2f, time: %.2f',cost_path,time_path);
% htitle = title(txt_title);
% set(htitle,'fontsize',15);

xmin    = min(min(map.X));
xmax    = max(max(map.X));
ymin    = min(min(map.Y));
ymax    = max(max(map.Y));

%% plot layers
H_layer = 8;
z0_up = 0.1;
z_layers = H_layer*(0:1:(DP.num_Z));

alpha_obs = 0.6;
alpha_ri = 0.7;

for nidx_z = 1:1:size(z_layers,2)
    if(nidx_z == 1)
        % plot colormap
        if strcmp(cost_type,'costmap')
            C_height = max(max(map.C)) - min(min(map.C));
            C_scale = 2.5 / C_height;
            C_plot_ = map.C * C_scale;
            C_plot = C_plot_ - max(max(C_plot_));
            surf(map.X,map.Y,C_plot,'EdgeColor','none','LineStyle','none','FaceLighting','phong');
            zmin_plot = min(min(C_plot));
        else
            zmin_plot = 0;
        end
        
        % plot obstacle
        for nidx_o = 1:1:size(DP.Obs,2)
            x = DP.Obs{nidx_o}.p(:,1)';
            y = DP.Obs{nidx_o}.p(:,2)';
            patch_x     = [x x(1)];
            patch_y     = [y y(1)];
            patch_z     = z_layers(1,nidx_z)*ones(size(patch_x));
            
            hobs    = patch(patch_x,patch_y,patch_z,obs_color);
            set(hobs,'EdgeColor','none','FaceAlpha',alpha_obs);
        end
        
        for nidx_d = 1:1:DP.num_D
            points      = DP.D{nidx_d}.p;
            label       = DP.D{nidx_d}.label;
            patch_x     = [points(:,1); points(1,1)];
            patch_y     = [points(:,2); points(1,2)];
            patch_z     = z_layers(1,nidx_z)*ones(size(patch_x));
            p_mean      = [mean(points,1) z_layers(1,nidx_z)+z0_up];
            label       = DP.D{nidx_d}.label;
            
            if(nidx_z == 1)
                patch_z = patch_z + z0_up;
            end
            
            % plot region of interest
            if(nidx_d >= DP.index_RI)
                hcolor_fill  = rgb('BlueViolet');
                h_ri    = patch(patch_x,patch_y,patch_z,hcolor_fill);
                set(h_ri,'EdgeColor','none','FaceAlpha',alpha_ri);
                hcolor_txt  = rgb('Azure');
                text(p_mean(1)-0.2,p_mean(2),p_mean(3)+0.25, ...
                    label,'FontSize',15,'FontWeight','Bold','Color',hcolor_txt,...
                    'HorizontalAlignment','center','VerticalAlignment','middle');
            end
            
            % plot mesh-edge
            % plot3(patch_x,patch_y,patch_z,'-','LineWidth',0.01,'Color',rgb('Silver'));
        end
    else
        % plot obstacle
        for nidx_o = 1:1:size(DP.Obs,2)
            x = DP.Obs{nidx_o}.p(:,1)';
            y = DP.Obs{nidx_o}.p(:,2)';
            patch_x     = [x x(1)];
            patch_y     = [y y(1)];
            patch_z     = z_layers(1,nidx_z)*ones(size(patch_x));
            
            hobs    = patch(patch_x,patch_y,patch_z,obs_color);
            set(hobs,'EdgeColor','none','FaceAlpha',alpha_obs);
        end
        
        for nidx_d = 1:1:DP.num_D
            points      = DP.D{nidx_d}.p;
            
            patch_x     = [points(:,1); points(1,1)];
            patch_y     = [points(:,2); points(1,2)];
            patch_z     = z_layers(1,nidx_z)*ones(size(patch_x));
            p_mean      = [mean(points,1) z_layers(1,nidx_z)+z0_up];
            label       = DP.D{nidx_d}.label;
            
            % plot region of interest
            if(nidx_d >= DP.index_RI)
                hcolor  = rgb('BlueViolet');
                h_ri    = patch(patch_x,patch_y,patch_z,hcolor);
                set(h_ri,'EdgeColor','none','FaceAlpha',alpha_ri);
                
                if nidx_z == size(z_layers,2)
                    hcolor_txt  = rgb('Azure');
                    text(p_mean(1)-0.2,p_mean(2),p_mean(3)+0.15, ...
                        label,'FontSize',15,'FontWeight','Bold','Color',hcolor_txt,...
                        'HorizontalAlignment','center','VerticalAlignment','middle');
                end
            end
                        
            % plot high-level plan
            if(nidx_z == (z_sel + 1))
                [LIA,~] = ismember(nidx_d,highlevel_plan);
                if(LIA)
                    hp = patch(patch_x,patch_y,patch_z,rgb('Salmon'));
                    set(hp,'EdgeColor',bcolor,'linewidth',0.5,'FaceAlpha',0.7);
                    
                    if(nidx_d == g_target)
                        plot3(patch_x,patch_y,patch_z,'-','color',rgb('Red'),'LineWidth',2.5);
                    end
                end
            end
            
            % plot initial mesh
            if(nidx_d == d_sel && nidx_z == (z_sel+1))
                plot3(patch_x,patch_y,patch_z,'-','color',rgb('Orange'),'LineWidth',2.5);
            end
            
            % plot mesh-edge
            plot3(patch_x,patch_y,patch_z,'-','LineWidth',0.3,'Color',mcolor);
        end
    end
end

%% plot bar
pseg_bar1 = [xmin ymin z_layers(1); xmin ymin z_layers(end)]';
pseg_bar2 = [xmax ymin z_layers(1); xmax ymin z_layers(end)]';
pseg_bar3 = [xmax ymax z_layers(1); xmax ymax z_layers(end)]';
pseg_bar4 = [xmin ymax z_layers(1); xmin ymax z_layers(end)]';
plot3(pseg_bar1(1,:),pseg_bar1(2,:),pseg_bar1(3,:),':','LineWidth',1.5,'Color',bcolor);
plot3(pseg_bar2(1,:),pseg_bar2(2,:),pseg_bar2(3,:),':','LineWidth',1.5,'Color',bcolor);
plot3(pseg_bar3(1,:),pseg_bar3(2,:),pseg_bar3(3,:),':','LineWidth',1.5,'Color',bcolor);
plot3(pseg_bar4(1,:),pseg_bar4(2,:),pseg_bar4(3,:),':','LineWidth',1.5,'Color',bcolor);

for nidx_d = DP.index_RI:1:DP.num_D
    points       = DP.D{nidx_d}.p;
    for nidx_p = 1:1:size(points,1)
        px = points(nidx_p,1);
        py = points(nidx_p,2);
        pbar_vertical = [px py z_layers(1)+z0_up; px py z_layers(end)]';
        plot3(pbar_vertical(1,:),pbar_vertical(2,:),pbar_vertical(3,:),':','LineWidth',1.0,'Color',rgb('BlueViolet'));
    end
end

%% plot border
for nidx_z = 1:1:size(z_layers,2)
    pseg_border = [xmin ymin z_layers(nidx_z); xmax ymin z_layers(nidx_z); ...
        xmax ymax z_layers(nidx_z); xmin ymax z_layers(nidx_z); xmin ymin z_layers(nidx_z)]';
    plot3(pseg_border(1,:),pseg_border(2,:),pseg_border(3,:),'-','LineWidth',1.5,'Color',bcolor);
end

%% plot edge & vertex
for nidx_t = 1:1:T.num_node
    if(nidx_t>1)
        pseg_ws = T.v.pseg{nidx_t}(idx_ws,:);
        z0_seg = z_layers(T.v.alpha(1,nidx_t)+1);
        zseg_ws = (z0_seg+z0_up)*ones(1,size(pseg_ws,2));
        pseg_ws_plot = [pseg_ws; zseg_ws];
        
        % plot edge
        if(T.e.rewire(1,nidx_t) == 1)
            plot3(pseg_ws(1,:),pseg_ws(2,:),z0_up*ones(1,size(pseg_ws,2)),'-','Color',rgb('Crimson'),'linewidth',0.5);
            plot3(pseg_ws_plot(1,:),pseg_ws_plot(2,:),pseg_ws_plot(3,:),'-','Color',rgb('Crimson'),'linewidth',0.5);
        else
            plot3(pseg_ws(1,:),pseg_ws(2,:),z0_up*ones(1,size(pseg_ws,2)),'-','Color',rgb('LightSkyBlue'),'linewidth',0.5);
            plot3(pseg_ws_plot(1,:),pseg_ws_plot(2,:),pseg_ws_plot(3,:),'-','Color',rgb('LightSkyBlue'),'linewidth',0.5);
        end
        
        plot3(pseg_ws_plot(1,1),pseg_ws_plot(2,1),pseg_ws_plot(3,1),'o','MarkerSize',0.33,'LineWidth',1,...
            'MarkerFaceColor',rgb('OliveDrab'),'MarkerEdgeColor',rgb('LightCyan'));
    end
end

%% plot target vertex
if(~isempty(v_target))
    v_target_plot = [v_target(1) v_target(2) z_layers(1+z_sel)];
    plot3(v_target_plot(1),v_target_plot(2),v_target_plot(3),'x','MarkerSize',4,'LineWidth',1.2,'Color',rgb('Red'));
end

%% plot initial vetex
x_init = T.v.x(1:2,1);
plot3(x_init(1),x_init(2),z0_up,'s','linewidth',0.75,'color',rgb('LimeGreen'),'MarkerSize',4);
plot3(x_init(1),x_init(2),z_layers(2)+z0_up,'s','linewidth',1.2,'color',rgb('LimeGreen'),'MarkerSize',4);

%% plot path
if(~isempty(path))
    path_ws = path(idx_ws,:);
%     for nidx_z = 1:1:DP.num_Z
%         [~,idx_found_z] = find(automaton_path == nidx_z);
%         z0_seg = z_layers(nidx_z+1) + z0_up;
%         path_per_z = [path_ws(:,idx_found_z); z0_seg*ones(1,size(idx_found_z,2))];
%         % plot3(path_per_z(1,:),path_per_z(2,:),path_per_z(3,:),'-','linewidth',4,'color',rgb('Yellow'));
%         % plot3(path_per_z(1,:),path_per_z(2,:),path_per_z(3,:),'-','linewidth',3,'color',rgb('OrangeRed'));
%         plot3(path_per_z(1,:),path_per_z(2,:),path_per_z(3,:),'-','linewidth',2.5,'color',rgb('Red'));
%     end
    % plot3(path_ws(1,:),path_ws(2,:),zeros(1,size(path_ws,2)),'-','linewidth',4,'color',rgb('Yellow'));
    % plot3(path_ws(1,:),path_ws(2,:),zeros(1,size(path_ws,2)),'-','linewidth',3,'color',rgb('OrangeRed'));
        
    plot3(path_ws(1,:),path_ws(2,:),z0_up*ones(1,size(path_ws,2)),'-','linewidth',1.2,'color',rgb('OrangeRed'));
end

if(is_long && ~isempty(traj_seg))
    z_ts = (z_layers(z_sel+1)+z0_up)*ones(1,size(traj_seg,2));
    traj_set_plot = [traj_seg; z_ts];
    plot3(traj_set_plot(1,:),traj_set_plot(2,:),traj_set_plot(3,:),'o','MarkerSize',1,'LineWidth',1,'color',rgb('OrangeRed'));
    plot3(traj_set_plot(1,:),traj_set_plot(2,:),z0_up*ones(1,size(traj_set_plot,2)),'o','MarkerSize',1,'LineWidth',1,'color',rgb('OrangeRed'));
end

xlim([map.xmin, map.xmax])
ylim([map.ymin, map.ymax])
zlim([zmin_plot-0.2, max(z_layers)+z0_up+0.2])
set(gca,'layer','top');
set(gca,'LooseInset',get(gca,'TightInset'));
view([-40 50]);
axis off;

% black
if is_black; set(hFig,'Color','k'); end
set(gcf, 'InvertHardCopy','off');
end

