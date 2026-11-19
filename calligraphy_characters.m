function characters = calligraphy_characters(value)
% UTF-16 单字切分，兼容扩展区汉字；首版输入连续文字，不接受空白。
value=char(value); assert(isrow(value) && ~isempty(value), ...
    'Calligraphy:Character','请输入要书写的汉字。');
assert(~any(isspace(value)),'Calligraphy:Character','请输入连续汉字；每行字数由排版参数控制，无需空格或换行。');
characters={}; i=1;
while i<=numel(value)
    width=1; code=double(value(i));
    if code>=55296 && code<=56319
        assert(i<numel(value) && double(value(i+1))>=56320 && double(value(i+1))<=57343, ...
            'Calligraphy:Character','输入包含无效的 Unicode 字符。'); width=2;
    elseif code>=56320 && code<=57343
        error('Calligraphy:Character','输入包含无效的 Unicode 字符。');
    end
    characters{end+1}=value(i:i+width-1); i=i+width; %#ok<AGROW>
end
end
