function name = calligraphy_output_name(glyph)
codes=arrayfun(@(code)sprintf('U%05X',code),glyph.codepoint,'UniformOutput',false);
name=strjoin(codes,'_');
if numel(codes)>1, name=['TEXT_' name]; end
if numel(name)>150
    digest=java.security.MessageDigest.getInstance('SHA-256');
    bytes=typecast(digest.digest(unicode2native(glyph.character,'UTF-8')),'uint8');
    key=lower(reshape(dec2hex(bytes,2).',1,[])); name=['TEXT_' key(1:24)];
end
end
