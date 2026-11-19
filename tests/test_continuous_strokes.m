function report = test_continuous_strokes
% 连续行笔回归：无内部零速、C2 几何、深度跟随、完整 IK 与导出。
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root);
characters={'一','人','永'}; report=struct([]);
for k=1:numel(characters)
    cfg=calligraphy_config(characters{k});
    result=plan_calligraphy(cfg);
    tr=result.trajectory; glyph=result.glyph;
    assert(result.validation.internalStops==0);
    assert(sum(strcmp({result.pieces.phase},'write'))==numel(glyph.strokes));
    assert(max(abs(glyph.bounds),[],'all')<=31.5+1e-9);
    for i=1:numel(glyph.strokes)
        st=glyph.strokes{i};
        assert(st.smoothingErrorMm<=cfg.strokeSmoothingToleranceMm);
        % 切向和弧长曲率连续（原始 C 段内部也没有强制停车）。
        for j=1:numel(st.segments)-1
            [p,v,a]=endpoint(st.segments(j).B,1);
            [p2,v2,a2]=endpoint(st.segments(j+1).B,0);
            assert(norm(p-p2)<1e-8 && norm(v-v2)<1e-7 && norm(a-a2)<1e-5);
        end
        ix=tr.strokeId==i & tr.phase=="write" & tr.progress>1e-9 & tr.progress<1-1e-9;
        assert(all(vecnorm(tr.cartesianVelocity(ix,:),2,2)>0));
        expected=calligraphy_depth(result.config.depthProfiles{i},tr.progress(ix),cfg.maxDepth);
        assert(max(abs(expected-tr.depth(ix)))<1e-10);
    end
    ends=round(tr.pieceEndTimes/cfg.Ts)+1;
    assert(max(abs(tr.cartesianVelocity(ends,:)),[],'all')<1e-7);
    assert(max(abs(tr.cartesianAcceleration(ends,:)),[],'all')<1e-7);
    % 解析笛卡尔速度与位置数值差分一致，排除只修改显示速度的假连续。
    numeric=(tr.position(3:end,:)-tr.position(1:end-2,:))/(2*cfg.Ts);
    assert(max(vecnorm(numeric-tr.cartesianVelocity(2:end-1,:),2,2))<.1);
    % 连续版本沿用常规导出位置，覆盖同字旧结果。
    folder=fullfile(root,'results',sprintf('U%05X',double(characters{k})));
    export_calligraphy(result,folder);
    points=readmatrix(fullfile(folder,'calligraphy_joint_points.txt'));
    assert(isequal(size(points),size(tr.q)));
    loaded=calligraphy_load_config(fullfile(folder,'config.json'));
    assert(isequal(loaded.depthProfiles,result.config.depthProfiles));
    report(k).character=characters{k}; report(k).duration=tr.t(end);
    report(k).samples=numel(tr.t); report(k).validation=result.validation;
    if strcmp(characters{k},'人')
        [fig,~]=plot_calligraphy_end_velocity(result);
        exportgraphics(fig,fullfile(root,'tests','continuous_velocity_preview.png'),'Resolution',120); close(fig);
    end
end
% 旧配置自动补齐新增的平滑容差；速度变化不改变同进度的提按。
cfg=rmfield(calligraphy_config('人'),'strokeSmoothingToleranceMm');
[cfg,glyph,pieces]=calligraphy_prepare(cfg);
fast=calligraphy_sample(cfg,glyph,pieces);
for k=1:numel(pieces), pieces(k).duration=2*pieces(k).duration; end
slow=calligraphy_sample(cfg,glyph,pieces);
assert(max(abs(fast.paperPosition-slow.paperPosition(1:2:end,:)),[],'all')<1e-8);
app=calligraphy_app(cfg); drawnow; delete(app.figure);
save(fullfile(root,'tests','continuous_test_report.mat'),'report');
file=fopen(fullfile(root,'tests','continuous_test_report.json'),'wt','n','UTF-8');
cleanup=onCleanup(@()fclose(file)); fprintf(file,'%s',jsonencode(report,'PrettyPrint',true));
fprintf('CONTINUOUS STROKE TESTS PASSED. SIMULINK NOT RUN.\n');
end

function [p,tangent,curvature]=endpoint(B,u)
[p,v,a]=calligraphy_bezier(B,u); g=norm(v);
tangent=v/g; curvature=a/g^2-v*dot(v,a)/g^4;
end
