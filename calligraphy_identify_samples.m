function tr = calligraphy_identify_samples(tr,glyph)
% strokeId 保持全局编号兼容旧程序；附加字符位置和字内笔画编号。
tr.characterIndex=zeros(size(tr.strokeId)); tr.localStrokeId=zeros(size(tr.strokeId));
valid=tr.strokeId>0;
tr.characterIndex(valid)=glyph.strokeCharacterIndex(tr.strokeId(valid));
tr.localStrokeId(valid)=glyph.localStrokeId(tr.strokeId(valid));
tr.characters=glyph.characters;
end
