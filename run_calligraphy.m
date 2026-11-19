%% 离线入口：计算并导出，不运行 Simulink。
cfg = calligraphy_config('人');
% 在此设置纸面、毛笔 TCP、下压深度等实测参数，或从交互界面保存的 JSON 读取：
% cfg = calligraphy_load_config('my_config.json');
result = plan_calligraphy(cfg);
outputFolder = export_calligraphy(result);
t=result.trajectory.t; q=result.trajectory.q;
qd=result.trajectory.qd; qdd=result.trajectory.qdd;
fprintf('TXT / CSV / MAT 已保存：%s\n',outputFolder);
% 需要仿真时，由你手动执行以下命令，然后在 Simulink 中点击 Run：
% prepare_calligraphy_simulink(result)
