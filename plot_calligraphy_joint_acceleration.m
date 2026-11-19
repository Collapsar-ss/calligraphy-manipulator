function [fig,axesList] = plot_calligraphy_joint_acceleration(result)
% J1～J6 规划角加速度分图；输入 result 或已导出的 MAT 文件路径。
if ischar(result) || isstring(result)
    saved=load(result,'result'); result=saved.result;
end
tr=result.trajectory;
assert(size(tr.qdd,2)==6 && size(tr.qdd,1)==numel(tr.t), ...
    'Calligraphy:Plot','关节加速度数据与时间不匹配。');
fig=figure('Name',sprintf('%s | 六轴关节加速度',result.config.character), ...
    'NumberTitle','off','Color','w','Position',[110 80 1200 800]);
layout=tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
axesList=gobjects(6,1); colors=lines(6); acceleration=rad2deg(tr.qdd);
starts=find([true;diff(tr.strokeId)~=0]);
starts=starts(tr.strokeId(starts)>0);
for j=1:6
    ax=nexttile(layout); axesList(j)=ax;
    plot(ax,tr.t,acceleration(:,j),'Color',colors(j,:),'LineWidth',1.1);
    grid(ax,'on'); xlabel(ax,'时间 (s)'); ylabel(ax,'角加速度 (deg/s²)');
    peak=max(abs(acceleration(:,j)));
    title(ax,sprintf('J%d | 绝对峰值 %.3f deg/s² | 上限 %.3f deg/s²', ...
        j,peak,result.config.jointAccelDeg(j)));
    xlim(ax,[tr.t(1) tr.t(end)]);
    for k=2:numel(starts)
        xline(ax,tr.t(starts(k)),':','Color',[.65 .65 .65],'HandleVisibility','off');
    end
    % 按实际峰值缩放，不把近似静止关节的浮点噪声放大成明显运动。
    span=max(.01,peak*1.12); ylim(ax,[-span span]);
end
linkaxes(axesList,'x');
title(layout,sprintf('%s · 规划关节角加速度 · %.3f s（虚线为下一笔落笔阶段开始）', ...
    result.config.character,tr.t(end)));
end
