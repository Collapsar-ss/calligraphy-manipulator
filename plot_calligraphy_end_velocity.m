function [fig,axesList] = plot_calligraphy_end_velocity(result)
% 笔尖 TCP 在基坐标系下的规划线速度；输入 result 或导出的 MAT 路径。
if ischar(result) || isstring(result)
    saved=load(result,'result'); result=saved.result;
end
tr=result.trajectory; velocity=tr.cartesianVelocity;
assert(size(velocity,2)==3 && size(velocity,1)==numel(tr.t), ...
    'Calligraphy:Plot','末端速度数据与时间不匹配。');
speed=vecnorm(velocity,2,2);
fig=figure('Name',sprintf('%s | 末端笔尖速度',result.config.character), ...
    'NumberTitle','off','Color','w','Position',[120 100 1150 700]);
layout=tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');
axesList=gobjects(2,1);
axesList(1)=nexttile(layout);
plot(axesList(1),tr.t,velocity,'LineWidth',1.1);
ylabel(axesList(1),'线速度 (mm/s)');
legend(axesList(1),{'Vx','Vy','Vz'},'Location','best');
title(axesList(1),'笔尖在机械臂基坐标系下的三个速度分量');
axesList(2)=nexttile(layout);
plot(axesList(2),tr.t,speed,'k','LineWidth',1.2);
hold(axesList(2),'on');
plot(axesList(2),tr.t,tr.speedLimit,'r--','LineWidth',1);
if isfield(tr,'localSpeedLimit')
    plot(axesList(2),tr.t,tr.localSpeedLimit,'Color',[.1 .55 .25],'LineStyle',':','LineWidth',1);
    legend(axesList(2),{'实际规划合速度 |V|','设定速度上限','路径节点限速参考'},'Location','best');
else
    legend(axesList(2),{'合速度 |V|','当前阶段速度上限'},'Location','best');
end
ylabel(axesList(2),'合速度 (mm/s)');
title(axesList(2),sprintf('合速度峰值 %.3f mm/s（包含书写、提按与空移）',max(speed)));
for j=1:2
    ax=axesList(j); grid(ax,'on'); xlabel(ax,'时间 (s)'); xlim(ax,[tr.t(1) tr.t(end)]);
    starts=find([true;diff(tr.strokeId)~=0]);
    starts=starts(tr.strokeId(starts)>0);
    for k=2:numel(starts)
        xline(ax,tr.t(starts(k)),':','Color',[.65 .65 .65],'HandleVisibility','off');
    end
end
linkaxes(axesList,'x');
title(layout,sprintf('%s · 笔尖 TCP 规划线速度 · %.3f s（虚线为下一笔落笔阶段开始）', ...
    result.config.character,tr.t(end)));
end
