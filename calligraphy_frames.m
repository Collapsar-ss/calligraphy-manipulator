function frames = calligraphy_frames(cfg)
% R = Rz(rz)*Ry(ry)*Rx(rx). 模型末端不能直接当作笔尖。
frames.paperR = rotation(cfg.paperRpyDeg);
frames.tipR = frames.paperR * rotation(cfg.brushRpyDeg);
frames.modelToTip = [rotation(cfg.toolRpyDeg),cfg.toolOffsetMm(:);0 0 0 1];
frames.modelR = frames.tipR * frames.modelToTip(1:3,1:3).';
frames.origin = cfg.paperOrigin(:).';
end

function R = rotation(degrees)
a = deg2rad(degrees);
cx=cos(a(1)); sx=sin(a(1)); cy=cos(a(2)); sy=sin(a(2)); cz=cos(a(3)); sz=sin(a(3));
R = [cz -sz 0;sz cz 0;0 0 1]*[cy 0 sy;0 1 0;-sy 0 cy]*[1 0 0;0 cx -sx;0 sx cx];
end
