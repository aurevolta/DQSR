function blkStruct=slblocks

% This function specifies that the library should appear
% in the Library Browser
% and be cached in the browser repository

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

Browser.Library = 'quat_matrix';
% 'mylib' is the name of the library

Browser.Name = 'Quaternion Matrix';
% 'My Library' is the library name that appears in the Library Browser

blkStruct.Browser = Browser;

end