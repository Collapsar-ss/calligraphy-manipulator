function [basePoint,paperPoint] = calligraphy_safe_point(cfg)
% 安全起止点采用基坐标：X/Y 固定，Z 随纸面中心高度自动更新。
basePoint=[370,0,cfg.paperOrigin(3)+100];
frames=calligraphy_frames(cfg);
paperPoint=(basePoint-frames.origin)*frames.paperR;
end
