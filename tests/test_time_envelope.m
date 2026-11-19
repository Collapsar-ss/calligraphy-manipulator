function test_time_envelope
% 长直线应达到设定巡航速度，局部降速不改动远处速度上限。
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root);
law.distance=(0:.25:200).'; law.speedCap=10*ones(size(law.distance));
law.accelerationCap=20*ones(size(law.distance)); law.referenceSpeed=law.speedCap;
law=calligraphy_time_law(law);
t=linspace(0,law.time(end),20001).';
[r,v,a]=calligraphy_time_at(law,t,law.time(end));
assert(all(diff(r)>=0) && all(v>=0) && max(v)<=10+1e-8 && max(v)>9.99);
assert(max(abs(a))<=20+1e-7);
assert(abs(r(1))<1e-12 && abs(r(end)-200)<1e-9);
piece.timeLaw=law; piece.duration=law.time(end);
tr.pieceId=1; tr.localIntervalId=400;
cfg=calligraphy_config;
updated=calligraphy_refine_timing(piece,tr,2,cfg);
assert(updated.timeLaw.speedCap(400)<law.speedCap(400));
assert(updated.timeLaw.speedCap(50)==law.speedCap(50));
assert(updated.timeLaw.velocity(50)==law.velocity(50));
assert(all(updated.timeLaw.velocity(2:end-1)>0));
assert(updated.duration>piece.duration);
fprintf('LOCAL ENVELOPE / LOCAL REFINEMENT TEST PASSED.\n');
end
