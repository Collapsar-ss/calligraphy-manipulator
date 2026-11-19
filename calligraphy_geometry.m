function glyph = calligraphy_geometry(cfg)
% 从固定版本的字库提取 M/C 路径，中心线整字等比缩放。
persistent xml sourcePath sourceStamp
root=fileparts(mfilename('fullpath'));
file=fullfile(root,'data','hanzivg.xml');
assert(isfile(file),'Calligraphy:Data','缺少字库：%s',file);
stamp=dir(file);
if isempty(xml) || ~strcmp(sourcePath,file) || sourceStamp~=stamp.datenum
    xml=fileread(file); sourcePath=file; sourceStamp=stamp.datenum;
end
chars=double(char(cfg.character));
if numel(chars)==2 && chars(1)>=55296 && chars(1)<=56319 && chars(2)>=56320 && chars(2)<=57343
    code=65536+(chars(1)-55296)*1024+chars(2)-56320;
elseif numel(chars)==1 && ~(chars>=55296 && chars<=57343)
    code=chars;
else, error('Calligraphy:Character','请输入一个汉字。'); end
key=sprintf('<kanji id="kvg:kanji_%05x">',code);
start=strfind(xml,key);
assert(numel(start)==1,'Calligraphy:MissingCharacter','字库中未找到该字符 U+%04X。',code);
tail=xml(start:end); stop=strfind(tail,'</kanji>'); block=tail(1:stop(1)+7);
paths=regexp(block,'<path\s+id="[^"]*-s(\d+)"\s+d="([^"]+)"','tokens');
assert(~isempty(paths),'Calligraphy:Data','汉字没有中心线路径。');
ids=cellfun(@(p)str2double(p{1}),paths); [ids,order]=sort(ids); paths=paths(order);
assert(isequal(ids,1:numel(ids)),'Calligraphy:Data','笔顺编号不连续。');
raw=cell(1,numel(paths)); bounds=zeros(0,2);
for i=1:numel(paths)
    d=paths{i}{2}; commands=regexp(d,'[A-Za-z]','match');
    assert(strcmp(commands{1},'M') && all(strcmp(commands(2:end),'C')), ...
        'Calligraphy:Data','不支持的 SVG 命令；固定字库只允许 M/C。');
    nums=str2double(regexp(d,'[-+]?(?:\d*\.\d+|\d+\.?\d*)','match'));
    assert(mod(numel(nums)-2,6)==0,'Calligraphy:Data','Bézier 坐标数量无效。');
    p=nums(1:2); n=(numel(nums)-2)/6; curves=zeros(4,2,n);
    for j=1:n
        next=reshape(nums(3+6*(j-1):2+6*j),2,3).';
        B=[p;next]; B(:,2)=-B(:,2); curves(:,:,j)=B;
        bounds=[bounds;extrema(B)]; %#ok<AGROW>
        p=next(3,:);
    end
    raw{i}=curves;
end
lo=min(bounds,[],1); hi=max(bounds,[],1); span=max(hi-lo);
assert(span>1e-9,'Calligraphy:Degenerate','字形长度为零。');
scale=(cfg.squareSizeMm-2*cfg.marginMm)/span; center=(lo+hi)/2;
glyph.character=cfg.character; glyph.codepoint=code; glyph.scale=scale;
glyph.bounds=[(lo-center)*scale;(hi-center)*scale];
glyph.strokes=cell(size(raw));
for i=1:numel(raw)
    source=raw{i}; segments=struct('B',{},'u',{},'ell',{},'length',{});
    for j=1:size(source,3)
        B=(source(:,:,j)-center)*scale;
        if sum(vecnorm(diff(B),2,2))<1e-9, continue; end
        [u,ell]=arcTable(B,cfg.geometryToleranceMm);
        if ell(end)<1e-9, continue; end
        segments(end+1)=struct('B',B,'u',u,'ell',ell,'length',ell(end)); %#ok<AGROW>
    end
    assert(~isempty(segments),'Calligraphy:Degenerate','第 %d 笔长度为零。',i);
    cumulative=[0 cumsum([segments.length])];
    original=struct('segments',segments,'cumulative',cumulative,'length',cumulative(end));
    glyph.strokes{i}=calligraphy_smooth_stroke(original,cfg);
end
% 平滑极值若超出字框，整字统一等比缩小以维持配置的尺寸约束。
smoothBounds=zeros(0,2);
for i=1:numel(glyph.strokes)
    for seg=glyph.strokes{i}.segments, smoothBounds=[smoothBounds;extrema(seg.B)]; end %#ok<AGROW>
end
extent=max(abs(smoothBounds),[],'all'); allowed=cfg.squareSizeMm/2-cfg.marginMm;
fit=min(1,allowed/max(extent,eps));
for i=1:numel(glyph.strokes)
    st=glyph.strokes{i};
    for j=1:numel(st.segments)
        st.segments(j).B=st.segments(j).B*fit;
        st.segments(j).ell=st.segments(j).ell*fit;
        st.segments(j).length=st.segments(j).length*fit;
    end
    st.cumulative=st.cumulative*fit; st.length=st.length*fit;
    glyph.strokes{i}=st;
end
glyph.smoothingFitScale=fit;
glyph.bounds=[min(smoothBounds,[],1);max(smoothBounds,[],1)]*fit;
end

function pts=extrema(B)
u=[0;1];
for axis=1:2
    a=-B(1,axis)+3*B(2,axis)-3*B(3,axis)+B(4,axis);
    b=2*(B(1,axis)-2*B(2,axis)+B(3,axis)); c=B(2,axis)-B(1,axis);
    r=roots([a b c]); r=real(r(abs(imag(r))<1e-10 & real(r)>0 & real(r)<1)); u=[u;r]; %#ok<AGROW>
end
pts=calligraphy_bezier(B,u);
end

function [u,ell]=arcTable(B,tol)
% 均匀参数细分 + Simpson 弧长积分。双层查表验证参数反解误差。
n=64;
while true
    u=linspace(0,1,n+1).'; um=(u(1:end-1)+u(2:end))/2;
    [~,v]=calligraphy_bezier(B,u); [~,vm]=calligraphy_bezier(B,um);
    speeds=vecnorm(v,2,2); mids=vecnorm(vm,2,2);
    lengths=(speeds(1:end-1)+4*mids+speeds(2:end))/(6*n);
    ell=[0;cumsum(lengths)];
    [keep,ix]=unique(ell,'stable');
    if numel(keep)<2, return; end
    inverse=pchip(keep,u(ix));
    quarter=(u(1:end-1)+um)/2;
    [~,vq]=calligraphy_bezier(B,quarter);
    half=(speeds(1:end-1)+4*vecnorm(vq,2,2)+mids)/(12*n);
    estimate=ppval(inverse,ell(1:end-1)+half);
    errorMm=max(vecnorm(calligraphy_bezier(B,estimate)-calligraphy_bezier(B,um),2,2));
    if errorMm<=tol/4 && max(lengths)<=.1
        u=u(ix); ell=keep; return
    end
    n=n*2;
    assert(n<=131072,'Calligraphy:Geometry','弧长查表未达到精度要求。');
end
end
