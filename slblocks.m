function blkStruct=slblocks

% Author: Aureliano Rivolta
% e-mail: aureliano.rivolta@polimi.it
% year: 2016
% private use only

% This function specifies that the library should appear
% in the Library Browser
% and be cached in the browser repository

%%

Browser.Library = 'quat_matrix';
% 'mylib' is the name of the library

Browser.Name = 'Quaternion Matrix';
% 'My Library' is the library name that appears in the Library Browser

blkStruct.Browser = Browser;

end