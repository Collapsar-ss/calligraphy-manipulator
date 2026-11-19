function [fig,axesList] = plot_calligraphy_joint_velocity(result)
% 六个关节速度分图；输入 result 或已导出的 MAT 文件路径。
if ischar(result) || isstring(result)
    saved=load(result,'result'); result=saved.result;
end
tr=result.trajectory;
assert(size(tr.qd,2)==6 && size(tr.qd,1)==numel(tr.t), ...
    'Calligraphy:Plot','关节速度数据与时间不匹配。');
fig=figure('Name',sprintf('%s | 六轴关节速度',result.config.character), ...
    'NumberTitle','off','Color','w','Position',[90 70 1200 800]);
layout=tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
axesList=gobjects(6,1); colors=lines(6); speed=rad2deg(tr.qd);
for j=1:6
    ax=nexttile(layout); axesList(j)=ax;
    plot(ax,tr.t,speed(:,j),'Color',colors(j,:),'LineWidth',1.2);
    grid(ax,'on'); xlabel(ax,'时间 (s)'); ylabel(ax,'角速度 (deg/s)');
    peak=max(abs(speed(:,j)));
    title(ax,sprintf('J%d | 峰值 %.3f deg/s | 上限 %.3f deg/s', ...
        j,peak,result.config.jointSpeedDeg(j)));
    xlim(ax,[tr.t(1) tr.t(end)]);
    % 相同竖线标识每笔起始落笔时刻，便于对应书写过程。
    starts=find([true;diff(tr.strokeId)~=0]);
    starts=starts(tr.strokeId(starts)>0);
    for k=2:numel(starts)
        xline(ax,tr.t(starts(k)),':','Color',[.65 .65 .65],'HandleVisibility','off');
    end
    % 近似静止轴避免将数值噪声放大成显著运动。
    span=max(.01,peak*1.12); ylim(ax,[-span span]);
end
linkaxes(axesList,'x');
title(layout,sprintf('%s · 规划关节速度 · %.3f s（虚线为下一笔落笔阶段开始）', ...
    result.config.character,tr.t(end)));
end
