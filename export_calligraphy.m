function folder = export_calligraphy(result,folder)
% 六列无表头空格分隔 TXT，和实验四格式一致。时间信息单独保存。
assert(result.validation.passed,'Calligraphy:Export','只能导出已经通过检查的轨迹。');
if nargin<2
    folder=fullfile(fileparts(mfilename('fullpath')),'results',calligraphy_output_name(result.glyph));
end
if ~isfolder(folder), mkdir(folder); end
tr=result.trajectory; cfg=result.config;
tipPose=[tr.position,repmat(rpy(tr.tipRotation),numel(tr.t),1)];
modelPose=[tr.modelPosition,repmat(rpy(tr.modelRotation),numel(tr.t),1)];
writePoints(fullfile(folder,'calligraphy_joint_points.txt'),rad2deg(tr.q));
writePoints(fullfile(folder,'calligraphy_cartesian_points.txt'),tipPose);
writePoints(fullfile(folder,'calligraphy_model_end_points.txt'),modelPose);
T=array2table([tr.t,tr.position,rad2deg(tr.q),rad2deg(tr.qd),rad2deg(tr.qdd),tr.strokeId,tr.progress,tr.depth], ...
    'VariableNames',{'time_s','tip_x_mm','tip_y_mm','tip_z_mm', ...
    'q1_deg','q2_deg','q3_deg','q4_deg','q5_deg','q6_deg', ...
    'qd1_deg_s','qd2_deg_s','qd3_deg_s','qd4_deg_s','qd5_deg_s','qd6_deg_s', ...
    'qdd1_deg_s2','qdd2_deg_s2','qdd3_deg_s2','qdd4_deg_s2','qdd5_deg_s2','qdd6_deg_s2', ...
    'stroke_id','progress','depth_mm'});
T.phase=tr.phase;
if isfield(tr,'characterIndex')
    T.character_index=tr.characterIndex; T.local_stroke_id=tr.localStrokeId;
    text=strings(numel(tr.t),1);
    for k=1:numel(result.glyph.characters), text(tr.characterIndex==k)=string(result.glyph.characters{k}); end
    T.character=text;
end
if isfield(tr,'localSpeedLimit')
    T.tip_speed_mm_s=vecnorm(tr.cartesianVelocity,2,2);
    T.configured_speed_limit_mm_s=tr.speedLimit;
    T.local_speed_reference_mm_s=tr.localSpeedLimit;
end
writetable(T,fullfile(folder,'calligraphy_timed_trajectory.csv'),'Encoding','UTF-8');
calligraphy_save_config(cfg,fullfile(folder,'config.json'));
t=tr.t; q=tr.q; qd=tr.qd; qdd=tr.qdd; %#ok<NASGU>
save(fullfile(folder,'calligraphy_trajectory.mat'),'result','t','q','qd','qdd','-v7.3');
meta=result.validation; meta.Ts=cfg.Ts; meta.samples=numel(t); meta.durationSeconds=t(end);
meta.source=result.source; meta.startJointDeg=rad2deg(q(1,:)); meta.endJointDeg=rad2deg(q(end,:));
if isfield(result.glyph,'characterCenters')
    meta.text=cfg.character; meta.characterCentersPaperMm=result.glyph.characterCenters;
    meta.charactersPerRow=cfg.charactersPerRow; meta.characterGapMm=cfg.characterGapMm;
end
writeText(fullfile(folder,'validation.json'),jsonencode(meta,'PrettyPrint',true));
status='UNCalibrated simulation example / 未标定示例';
if cfg.calibrated, status='User-marked calibrated / 用户标记已标定'; end
description=sprintf(['%s\nTs = %.12g s\nSamples = %d\nDuration = %.9g s\n' ...
    'TXT: no header, space separated, one row per sample.\n' ...
    'calligraphy_joint_points.txt: q1 q2 q3 q4 q5 q6 [deg].\n' ...
    'calligraphy_cartesian_points.txt: brush TCP X Y Z Rx Ry Rz [mm, deg].\n' ...
    'calligraphy_model_end_points.txt: original experiment-4 model end pose [mm, deg].\n' ...
    'Rotation convention: Rz(Rz)*Ry(Ry)*Rx(Rx).\n' ...
    'First/last TCP sample is [370, 0, paperOrigin(3)+100] mm in base coordinates.\n' ...
    'Includes safe-height transfer and vertical approach/retreat for the writing task.\n' ...
    'No motion from the real current pose to this safe start point is included.\n' ...
    'Configure the controller TCP to match the exported TCP pose; do not apply tool offset twice.\n' ...
    'Controller must execute one sample every Ts; internal replanning can change speed/depth timing.\n' ...
    'Negative depth in CSV means above paper, positive means pressed below contact zero.\n' ...
    'Simulink and brush contact/collision have NOT been tested by this exporter.\n'],status,cfg.Ts,numel(t),t(end));
writeText(fullfile(folder,'point_file_info.txt'),description);
end

function a=rpy(R)
y=atan2(-R(3,1),hypot(R(1,1),R(2,1)));
if abs(cos(y))>1e-9, x=atan2(R(3,2),R(3,3)); z=atan2(R(2,1),R(1,1));
else, x=0; z=atan2(-R(1,2),R(2,2)); end
a=rad2deg([x y z]);
end
function writePoints(file,p)
f=fopen(file,'wt'); assert(f>=0,'无法创建点位文件。'); cleanup=onCleanup(@()fclose(f));
fprintf(f,'%.8f %.8f %.8f %.8f %.8f %.8f\n',p.');
end
function writeText(file,text)
f=fopen(file,'wt','n','UTF-8'); assert(f>=0,'无法创建说明文件。'); cleanup=onCleanup(@()fclose(f)); fprintf(f,'%s',text);
end
