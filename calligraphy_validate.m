function cfg = calligraphy_validate(cfg)
% 配置错误在规划之前报出，不静默夹紧导入的深度。
if ~isfield(cfg,'strokeSmoothingToleranceMm'), cfg.strokeSmoothingToleranceMm=.05; end
required = fieldnames(calligraphy_config);
if ~isfield(cfg,'charactersPerRow'), cfg.charactersPerRow=3; end
if ~isfield(cfg,'characterGapMm'), cfg.characterGapMm=30; end
assert(all(isfield(cfg,required)), 'Calligraphy:Config', '配置缺少字段，请从 calligraphy_config 生成配置。');
positive = {'squareSizeMm','Ts','writeSpeed','travelSpeed','verticalSpeed', ...
    'cartAcceleration','verticalAcceleration','liftHeight','maxDepth','geometryToleranceMm','strokeSmoothingToleranceMm','maxSamples','maxTimeScalingIterations'};
for i=1:numel(positive)
    validateattributes(cfg.(positive{i}),{'numeric'},{'scalar','real','finite','positive'},'',positive{i});
end
validateattributes(cfg.marginMm,{'numeric'},{'scalar','real','finite','nonnegative'});
validateattributes(cfg.charactersPerRow,{'numeric'},{'scalar','integer','positive','finite'});
validateattributes(cfg.characterGapMm,{'numeric'},{'scalar','real','finite','nonnegative'});
assert(2*cfg.marginMm < cfg.squareSizeMm,'Calligraphy:Config','边距必须小于字框边长的一半。');
triples = {'paperOrigin','paperRpyDeg','brushRpyDeg','toolOffsetMm','toolRpyDeg'};
for i=1:numel(triples)
    validateattributes(cfg.(triples{i}),{'numeric'},{'vector','numel',3,'finite','real'});
    cfg.(triples{i})=reshape(cfg.(triples{i}),1,3);
end
six = {'qSeedDeg','jointSpeedDeg','jointAccelDeg','depthProgress','depthDefault'};
for i=1:numel(six)
    validateattributes(cfg.(six{i}),{'numeric'},{'vector','numel',6,'finite','real'});
    cfg.(six{i})=reshape(cfg.(six{i}),1,6);
end
assert(all(cfg.jointSpeedDeg>0) && all(cfg.jointAccelDeg>0),'Calligraphy:Config','关节速度和加速度上限必须为正。');
validateattributes(cfg.jointLimitsDeg,{'numeric'},{'size',[6 2],'finite','real'});
assert(all(cfg.jointLimitsDeg(:,1)<cfg.jointLimitsDeg(:,2)),'Calligraphy:Config','关节限位区间无效。');
assert(all(cfg.qSeedDeg>=cfg.jointLimitsDeg(:,1).' & cfg.qSeedDeg<=cfg.jointLimitsDeg(:,2).'),'Calligraphy:Config','逆解种子超出关节限位。');
assert(isscalar(cfg.calibrated) && ismember(cfg.calibrated,[0 1]),'Calligraphy:Config','calibrated 应为逻辑值。');
% JSON 同形 cell 数组在 jsondecode 后可能成为三维 numeric 数组。
if isnumeric(cfg.depthProfiles)
    a=cfg.depthProfiles;
    if isempty(a), cfg.depthProfiles={};
    elseif size(a,1)==6 && size(a,2)==2
        cfg.depthProfiles=arrayfun(@(k) a(:,:,k),1:size(a,3),'UniformOutput',false);
    elseif size(a,2)==6 && size(a,3)==2
        cfg.depthProfiles=arrayfun(@(k) squeeze(a(k,:,:)),1:size(a,1),'UniformOutput',false);
    else, error('Calligraphy:Depth','深度配置必须为每笔一个 6x2 数组。'); end
end
assert(iscell(cfg.depthProfiles),'Calligraphy:Depth','depthProfiles 必须为 cell 数组。');
calligraphy_depth([cfg.depthProgress(:),cfg.depthDefault(:)],0,cfg.maxDepth);
for i=1:numel(cfg.depthProfiles)
    calligraphy_depth(cfg.depthProfiles{i},0,cfg.maxDepth);
end
cfg.character=char(cfg.character);
calligraphy_characters(cfg.character);
end
