function calligraphy_save_config(cfg,file)
cfg=calligraphy_validate(cfg);
% 用带 strokeN 名称的结构体固定 JSON 形状，避免 cell 自动转换歧义。
profiles=struct;
for i=1:numel(cfg.depthProfiles), profiles.(sprintf('stroke%d',i))=cfg.depthProfiles{i}; end
cfg.depthProfiles=profiles;
f=fopen(file,'wt','n','UTF-8'); assert(f>=0,'Calligraphy:File','无法写入配置文件。');
cleanup=onCleanup(@()fclose(f)); fprintf(f,'%s',jsonencode(cfg,'PrettyPrint',true));
end
