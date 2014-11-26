function TimeStamp(name,varargin)
  save(sprintf('%s-%d',name,time),clock*[1e8 1e6 1e4 1e2 1 0].',varargin{:});
end