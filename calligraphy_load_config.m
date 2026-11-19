function cfg = calligraphy_load_config(file)
cfg=jsondecode(fileread(file));
if isfield(cfg,'depthProfiles') && isstruct(cfg.depthProfiles)
    profiles=cfg.depthProfiles; names=fieldnames(profiles); cfg.depthProfiles=cell(1,numel(names));
    for i=1:numel(names)
        field=sprintf('stroke%d',i); assert(isfield(profiles,field),'Calligraphy:Config','深度笔顺编号不连续。');
        cfg.depthProfiles{i}=profiles.(field);
    end
end
cfg=calligraphy_validate(cfg);
end
