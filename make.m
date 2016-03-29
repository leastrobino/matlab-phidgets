%
%  make.m
%
%  Created by Léa Strobino.
%  Copyright 2015 hepia. All rights reserved.
%

function make(varargin)

MEX = {};
switch computer('arch')
  case 'maci64'
    MEX = [MEX {'-I/Library/Frameworks/Phidget21.framework/Headers'}];
    MEX = [MEX {'LDFLAGS="$LDFLAGS -F/Library/Frameworks -framework Phidget21"'}];
    MEX = [MEX {'-largeArrayDims'}];
  case 'win32'
    MEX = [MEX {['-I"' cd '\SDK"']}];
    MEX = [MEX {['-L"' cd '\SDK\x86"']}];
    MEX = [MEX {'-lphidget21'}];
  case 'win64'
    MEX = [MEX {['-I"' cd '\SDK"']}];
    MEX = [MEX {['-L"' cd '\SDK\x64"']}];
    MEX = [MEX {'-lphidget21'}];
    MEX = [MEX {'-largeArrayDims'}];
end
MEX = [MEX {'-outdir','private'}];

if nargin == 0
  varargin = {'clean','all'};
end

warning('off','MATLAB:DELETE:FileNotFound');

if any(strcmpi(varargin,'clean'))
  m = mexext('all');
  for k = 1:length(m)
    delete(['*.' m(k).ext]);
    delete(['private/*.' m(k).ext]);
  end
  delete *.p
  delete private/*.p
end

if any(strcmpi(varargin,'all')) || any(strcmpi(varargin,'phidget21encoder'))
  mex('phidget21encoder.c',MEX{:});
end

if any(strcmpi(varargin,'all')) || any(strcmpi(varargin,'phidget21interface'))
  mex('phidget21interface.c',MEX{:});
end
