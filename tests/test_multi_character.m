function report=test_multi_character(reuseGenerated)
% 单个四字换行案例：排版、逐字编辑、配置重载、完整 IK 与导出。
if nargin<1, reuseGenerated=false; end
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root);
cfg=calligraphy_config('人人人人');
[cfg,glyph,pieces]=calligraphy_prepare(cfg);
expected=[-130 65;0 65;130 65;-130 -65];
assert(isequal(glyph.characterCenters,expected));
assert(numel(glyph.strokes)==8 && glyph.layoutRows==2 && glyph.layoutColumns==3);
assert(isequal(glyph.localStrokeId,[1 2 1 2 1 2 1 2]));
assert(sum(strcmp({pieces.phase},'safe_lower'))==1 && sum(strcmp({pieces.phase},'safe_lift'))==1);
app=calligraphy_app(cfg); cleanup=onCleanup(@()delete(app.figure));
app.characterSelector.Value=app.characterSelector.Items{4};
app.characterSelector.ValueChangedFcn(app.characterSelector,[]);
data=app.pointTable.Data; data(3,2)=1.5; app.pointTable.Data=data;
app.pointTable.CellEditCallback(app.pointTable,struct('Indices',[3 2]));
edited=app.getConfig(); assert(edited.depthProfiles{7}(3,2)==1.5 && edited.depthProfiles{1}(3,2)==1.2);
app.copySameCharacterButton.ButtonPushedFcn(app.copySameCharacterButton,[]);
copied=app.getConfig(); assert(isequal(copied.depthProfiles{1},copied.depthProfiles{7}));
file=fullfile(root,'tests','multi_character_config.json'); calligraphy_save_config(copied,file);
loaded=calligraphy_load_config(file); assert(isequal(loaded.depthProfiles,copied.depthProfiles));
drawnow; exportapp(app.figure,fullfile(root,'tests','multi_character_preview.png')); clear cleanup
if reuseGenerated
    saved=load(fullfile(root,'results',calligraphy_output_name(glyph),'calligraphy_trajectory.mat'),'result');
    result=saved.result;
else
    result=plan_calligraphy(copied);
end
tr=result.trajectory;
for i=1:4
    ix=tr.characterIndex==i & tr.phase=="write";
    assert(any(ix) && all(ismember(tr.localStrokeId(ix),[1 2])));
end
assert(all(tr.characterIndex([1 end])==0));
safe=calligraphy_safe_point(copied);
assert(max(abs(tr.position([1 end],:)-safe),[],'all')<1e-9);
folder=export_calligraphy(result);
points=readmatrix(fullfile(folder,'calligraphy_joint_points.txt'));
assert(isequal(size(points),size(tr.q)));
csv=fullfile(folder,'calligraphy_timed_trajectory.csv');
options=detectImportOptions(csv,'Encoding','UTF-8'); options=setvartype(options,'character','string');
timed=readtable(csv,options);
assert(isequal(timed.character_index,tr.characterIndex));
assert(all(timed.character(tr.characterIndex>0)=="人"));
% 单字目录和旧配置兼容；缺字能指出具体字符位置。
single=calligraphy_config('人'); single=rmfield(single,{'charactersPerRow','characterGapMm'});
[single,singleGlyph]=calligraphy_prepare(single); %#ok<ASGLU>
assert(strcmp(calligraphy_output_name(singleGlyph),'U04EBA'));
try
    bad=calligraphy_config(['人' char(1)]); calligraphy_prepare(bad);
    error('Missing-character check did not fail.');
catch err
    assert(strcmp(err.identifier,'Calligraphy:MissingCharacter') && contains(err.message,'第 2 个字'));
end
report=struct('passed',true,'text',copied.character,'centersPaperMm',expected, ...
    'durationSeconds',tr.t(end),'samples',numel(tr.t),'folder',folder,'simulinkRun',false);
f=fopen(fullfile(root,'tests','multi_character_report.json'),'wt','n','UTF-8');
cleanup=onCleanup(@()fclose(f)); fprintf(f,'%s',jsonencode(report,'PrettyPrint',true));
disp(report); fprintf('MULTI CHARACTER TEST PASSED. SIMULINK NOT RUN.\n');
end
