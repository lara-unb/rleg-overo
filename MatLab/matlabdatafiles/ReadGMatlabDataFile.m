function var = ReadGMatlabDataFile(varname, filename)

eval(sprintf('load(''%s'',''%s*'');',filename,varname));
eval(sprintf('varsize = %s_size;',varname));
eval(sprintf('s = whos(''%s_*'');',varname));
eval(sprintf('var = zeros(%i,1);',varsize));
datastartindex = 0;
ndatastructs = length(s)-1;
for ns=1:ndatastructs,
   startstrg = sprintf('%s_%i_',varname,datastartindex);
   eval(sprintf('s = whos(''%s*'');',startstrg));
   if(length(s)==1),
      dataendindex = str2num(s.name(length(startstrg)+1:length(s.name)));
      eval(sprintf('var(%i:%i) = %s%i;',datastartindex+1,dataendindex+1,startstrg,dataendindex));
      datastartindex = dataendindex+1;
   end
end,

%eval(['load ',filename,';']);
%eval(['varsize = ',varname,'_size;']);
%eval(['var = zeros(', num2str(varsize),',1);']);
%for n=1:varsize,
%	eval(['var(n) = ',varname,'_',num2str(n-1),';']);
%end,
