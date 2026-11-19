function report = run_calligraphy_tests(reuseGenerated)
% 仅数学/文件/界面检查，绝不调用 Simulink。
% 仅修改 UI 后，可传 true 复核已生成的默认结果，避免重复逐点逆解。
if nargin<1, reuseGenerated=false; end
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root,fullfile(root,'robot'));
report=struct('character',{},'samples',{},'duration',{},'maxFkError',{});
characters={'一','人','永'};
for c=1:numel(characters)
    cfg=calligraphy_config(characters{c});
    if reuseGenerated
        saved=fullfile(root,'results',sprintf('U%05X',double(characters{c})),'calligraphy_trajectory.mat');
        loadedResult=load(saved,'result'); result=loadedResult.result;
        fprintf('Rechecking existing verified trajectory: %s\n',characters{c});
    else
        result=plan_calligraphy(cfg);
    end
    assert(result.validation.passed && ~result.validation.simulinkRun);
    tr=result.trajectory;
    assert(max(abs(result.glyph.bounds(:)))<=31.5+1e-9);
    % 用独立自适应积分校验弧长反查，不只比较同一个查表算法。
    for stroke=1:numel(result.glyph.strokes)
        for segment=result.glyph.strokes{stroke}.segments
            for u=[.17 .43 .79]
                exact=integral(@(v)curveSpeed(segment.B,v),0,u,'AbsTol',1e-10);
                recovered=interp1(segment.ell,segment.u,exact,'pchip');
                errorMm=norm(calligraphy_bezier(segment.B,recovered)-calligraphy_bezier(segment.B,u));
                assert(errorMm<.01);
            end
        end
    end
    assert(all(abs(diff(tr.t)-cfg.Ts)<1e-9));
    assert(all(abs(tr.qd)<=deg2rad(cfg.jointSpeedDeg)*(1+1e-6),'all'));
    assert(all(abs(tr.qdd)<=deg2rad(cfg.jointAccelDeg)*(1+1e-6),'all'));
    assert(all(tr.q>=deg2rad(cfg.jointLimitsDeg(:,1).')-1e-10,'all'));
    assert(all(tr.q<=deg2rad(cfg.jointLimitsDeg(:,2).')+1e-10,'all'));
    boundary=round(tr.pieceEndTimes/cfg.Ts)+1;
    assert(max(abs(tr.cartesianVelocity(boundary,:)),[],'all')<1e-7);
    assert(max(abs(tr.cartesianAcceleration(boundary,:)),[],'all')<1e-7);
    safe=calligraphy_safe_point(cfg);
    assert(max(abs(tr.position([1 end],:)-safe),[],'all')<1e-9);
    for i=1:numel(result.glyph.strokes)
        ix=tr.strokeId==i & tr.phase=="write";
        expected=calligraphy_depth(result.config.depthProfiles{i},tr.progress(ix),cfg.maxDepth);
        assert(max(abs(expected-tr.depth(ix)))<1e-10);
    end
    folder=export_calligraphy(result);
    joint=readmatrix(fullfile(folder,'calligraphy_joint_points.txt'));
    pose=readmatrix(fullfile(folder,'calligraphy_cartesian_points.txt'));
    assert(isequal(size(joint),size(tr.q)) && size(pose,2)==6);
    assert(max(abs(joint-rad2deg(tr.q)),[],'all')<5.1e-9);
    assert(max(abs(pose(:,1:3)-tr.position),[],'all')<5.1e-9);
    loaded=calligraphy_load_config(fullfile(folder,'config.json'));
    assert(isequal(loaded.depthProfiles,result.config.depthProfiles));
    report(end+1)=struct('character',characters{c},'samples',numel(tr.t), ...
        'duration',tr.t(end),'maxFkError',max(tr.ikPositionError)); %#ok<AGROW>
end

% 非均匀控制点，极端深浅交替也不能过冲。
points=[0 0;.01 2;.2 .2;.8 1.7;.99 .1;1 0];
[d,ds,dss]=calligraphy_depth(points,linspace(0,1,10001),2);
assert(min(d)>=-1e-12 && max(d)<=2+1e-12 && all(isfinite([d ds dss])));
[d,ds,dss]=calligraphy_depth(points,points(:,1),2);
assert(max(abs(d-points(:,2)))<1e-10 && max(abs([ds;dss]))<1e-7);

% 不同速度不改变同一笔画进度对应的深度。
cfg=calligraphy_config('人'); [cfg,glyph,pieces]=calligraphy_prepare(cfg);
slow=pieces; for k=1:numel(slow), slow(k).duration=2*pieces(k).duration; end
fast=calligraphy_sample(cfg,glyph,pieces); slow=calligraphy_sample(cfg,glyph,slow);
assert(max(abs(fast.paperPosition-slow.paperPosition(1:2:end,:)),[],'all')<1e-9);

% 非水平纸面、非零工具偏移及工具旋转的正解回代。
cfg=calligraphy_config('一'); cfg.Ts=.02;
cfg.paperRpyDeg=[2 -3 15]; cfg.toolOffsetMm=[8 -5 90]; cfg.toolRpyDeg=[1 2 5];
tilted=plan_calligraphy(cfg,[]); assert(tilted.validation.maxPositionErrorMm<.01);
ix=tilted.trajectory.phase=="write"; frames=calligraphy_frames(cfg);
paper=(tilted.trajectory.position(ix,:)-frames.origin)*frames.paperR;
assert(max(abs(paper(:,3)+tilted.trajectory.depth(ix)))<1e-9);

cfg=calligraphy_config('一'); cfg.paperOrigin=[5000 0 400]; cfg.Ts=.05;
mustFail(@()plan_calligraphy(cfg,[]),'Calligraphy:IK');
cfg=calligraphy_config('不存在'); mustFail(@()calligraphy_geometry(cfg),'Calligraphy:Character');
cfg=calligraphy_config(char(1)); mustFail(@()calligraphy_geometry(cfg),'Calligraphy:MissingCharacter');
cfg=calligraphy_config; cfg.depthDefault(2)=3; mustFail(@()calligraphy_validate(cfg),'Calligraphy:Depth');
bad=points; bad(3,1)=bad(2,1); mustFail(@()calligraphy_depth(bad,.5,2),'Calligraphy:Depth');

% 真正构造 UI，并经表格/笔画回调修改数据、切换、保存与重新加载。
app=calligraphy_app; cleanup=onCleanup(@()delete(app.figure));
assert(height(app.pointTable.Data)==6);
p=app.pointTable.Data; p(3,:)=[45 1.6]; app.pointTable.Data=p;
app.pointTable.CellEditCallback(app.pointTable,struct('Indices',[3 2]));
edited=app.getConfig(); assert(edited.depthProfiles{1}(3,1)==.45 && edited.depthProfiles{1}(3,2)==1.6);
app.strokeSelector.Value=app.strokeSelector.Items{2};
app.strokeSelector.ValueChangedFcn(app.strokeSelector,[]);
assert(app.pointTable.Data(3,1)==40);
app.strokeSelector.Value=app.strokeSelector.Items{1}; app.strokeSelector.ValueChangedFcn(app.strokeSelector,[]);
assert(app.pointTable.Data(3,1)==45);
file=fullfile(root,'tests','edited_config.json'); calligraphy_save_config(app.getConfig(),file);
reloaded=calligraphy_load_config(file); assert(isequal(reloaded.depthProfiles,edited.depthProfiles));
drawnow; exportapp(app.figure,fullfile(root,'tests','editor_preview.png'));
clear cleanup
% 检查生成按钮返回后仍能编辑，并且任何编辑会让旧结果失效。
uiCfg=calligraphy_config('一'); uiCfg.Ts=.02; app2=calligraphy_app(uiCfg);
cleanup2=onCleanup(@()delete(app2.figure));
app2.generateButton.ButtonPushedFcn(app2.generateButton,[]);
assert(~isempty(app2.getResult()) && strcmp(app2.generateButton.Enable,'on'));
p=app2.pointTable.Data; p(2,2)=.7; app2.pointTable.Data=p;
app2.pointTable.CellEditCallback(app2.pointTable,struct('Indices',[2 2]));
assert(isempty(app2.getResult()) && strcmp(app2.exportButton.Enable,'off'));
clear cleanup2
save(fullfile(root,'tests','test_report.mat'),'report');
f=fopen(fullfile(root,'tests','test_report.json'),'wt','n','UTF-8');
fprintf(f,'%s',jsonencode(struct('passed',true,'simulinkRun',false,'characters',report),'PrettyPrint',true)); fclose(f);
disp(struct2table(report)); fprintf('ALL OFFLINE TESTS PASSED. SIMULINK NOT RUN.\n');
end

function v=curveSpeed(B,u)
[~,du]=calligraphy_bezier(B,u); v=reshape(vecnorm(du,2,2),size(u));
end

function mustFail(action,id)
try
    action();
catch err
    assert(strcmp(err.identifier,id),'Expected %s, got %s: %s',id,err.identifier,err.message); return
end
error('Expected error %s was not raised.',id);
end
