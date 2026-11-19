function pieces = calligraphy_refine_timing(pieces,tr,factor,cfg)
% 只降低超限位置附近的节点速度，重新传播加减速约束。
for k=1:numel(pieces)
    ix=tr.pieceId==k & factor>1+1e-6;
    if ~any(ix), continue; end
    law=pieces(k).timeLaw; n=numel(law.distance);
    perInterval=accumarray(tr.localIntervalId(ix),factor(ix),[n-1,1],@max,1);
    node=max([1;perInterval],[perInterval;1]);
    % 影响邻接多项式，增加一个节点的缓冲范围。
    spread=node; spread(2:end)=max(spread(2:end),node(1:end-1));
    spread(1:end-1)=max(spread(1:end-1),node(2:end));
    change=spread>1+1e-6;
    law.speedCap(change)=law.speedCap(change)./(spread(change)*1.08);
    law.referenceSpeed(change)=law.referenceSpeed(change)./(spread(change)*1.08);
    law=calligraphy_time_law(law); pieces(k).timeLaw=law;
    pieces(k).duration=ceil(law.time(end)/cfg.Ts)*cfg.Ts;
end
end
