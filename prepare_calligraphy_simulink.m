function prepare_calligraphy_simulink(result)
% 仅准备输入、打开模型，不 update，不 sim，不启动仿真，不保存原模型。
% 用法：prepare_calligraphy_simulink(result)，然后自行按 Run。
if ischar(result) || isstring(result)
    loaded=load(result,'result'); result=loaded.result;
end
assert(result.validation.passed,'Calligraphy:Simulation','轨迹检查尚未通过。');
tr=result.trajectory;
assignin('base','t',tr.t); assignin('base','q',tr.q);
assignin('base','qd',tr.qd); assignin('base','qdd',tr.qdd);
assignin('base','calligraphy_result',result);
root=fileparts(mfilename('fullpath'));
modelDir=fullfile(root,'..','实验四','实验四','Matlab代码','HL-0905');
assert(isfile(fullfile(modelDir,'HL0905_2.slx')),'Calligraphy:Simulation','未找到实验四 HL0905_2.slx。');
addpath(modelDir);
load_system(fullfile(modelDir,'HL0905_2.slx'));
set_param('HL0905_2','StopTime',sprintf('%.12g',tr.t(end)));
% 手动点击 Run 后自动打开 Mechanics Explorer 三维机械臂界面。
% 这里只修改当前内存中的模型设置，不保存实验四原模型，也不启动仿真。
set_param('HL0905_2','SimMechanicsOpenEditorOnUpdate','on');
converter=find_system('HL0905_2','LookUnderMasks','all','FollowLinks','on','Name','Simulink-PS Converterq');
if ~isempty(converter)
    set_param(converter{1},'FilteringAndDerivatives','filter','SimscapeFilterOrder','2', ...
        'InputFilterTimeConstant',sprintf('%.12g',result.config.Ts));
end
% 固定 STL 绝对路径，切换 MATLAB 当前目录后也能手动启动。
blocks=find_system('HL0905_2','LookUnderMasks','all','FollowLinks','on','Type','Block');
for i=1:numel(blocks)
    params=get_param(blocks{i},'DialogParameters');
    if isfield(params,'ExtGeomFileName')
        name=get_param(blocks{i},'ExtGeomFileName');
        [~,stem,ext]=fileparts(name); candidate=fullfile(modelDir,[stem ext]);
        if isfile(candidate), set_param(blocks{i},'ExtGeomFileName',candidate); end
    end
end
open_system('HL0905_2');
fprintf(['输入已准备，模型已打开；手动运行后将自动打开 Mechanics Explorer。' ...
    '本函数没有执行仿真。\n']);
end
