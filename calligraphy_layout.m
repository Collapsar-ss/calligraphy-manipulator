function glyph = calligraphy_layout(cfg)
% 横排从左向右、从上向下换行；paperOrigin 是整段排版外框中心。
persistent cache
if isempty(cache), cache=containers.Map('KeyType','char','ValueType','any'); end
characters=calligraphy_characters(cfg.character); n=numel(characters);
columns=min(cfg.charactersPerRow,n); rows=ceil(n/cfg.charactersPerRow);
pitch=cfg.squareSizeMm+cfg.characterGapMm;
width=columns*cfg.squareSizeMm+(columns-1)*cfg.characterGapMm;
height=rows*cfg.squareSizeMm+(rows-1)*cfg.characterGapMm;
centers=zeros(n,2); maps=[]; locals=[]; strokes={}; allBounds=zeros(0,2);
codepoints=zeros(1,n); fits=zeros(1,n);
for k=1:n
    single=cfg; single.character=characters{k};
    cacheKey=jsonencode({characters{k},cfg.squareSizeMm,cfg.marginMm, ...
        cfg.geometryToleranceMm,cfg.strokeSmoothingToleranceMm});
    try
        if isKey(cache,cacheKey), part=cache(cacheKey);
        else, part=calligraphy_geometry(single); cache(cacheKey)=part; end
    catch err
        error(err.identifier,'第 %d 个字“%s”：%s',k,characters{k},err.message);
    end
    if k==1, glyph=part; end
    column=mod(k-1,cfg.charactersPerRow); row=floor((k-1)/cfg.charactersPerRow);
    center=[-width/2+cfg.squareSizeMm/2+column*pitch, ...
        height/2-cfg.squareSizeMm/2-row*pitch]; centers(k,:)=center;
    for j=1:numel(part.strokes)
        st=part.strokes{j};
        for b=1:numel(st.segments), st.segments(b).B=st.segments(b).B+center; end
        strokes{end+1}=st; maps(end+1)=k; locals(end+1)=j; %#ok<AGROW>
    end
    codepoints(k)=part.codepoint; fits(k)=part.smoothingFitScale;
    allBounds=[allBounds;part.bounds+center]; %#ok<AGROW>
end
glyph.character=cfg.character; glyph.codepoint=codepoints; glyph.characters=characters;
glyph.characterCenters=centers; glyph.strokeCharacterIndex=maps;
glyph.localStrokeId=locals; glyph.strokes=strokes;
glyph.smoothingFitScale=fits; glyph.bounds=[min(allBounds,[],1);max(allBounds,[],1)];
glyph.layoutBounds=[-width/2,-height/2;width/2,height/2];
glyph.layoutRows=rows; glyph.layoutColumns=columns;
end
