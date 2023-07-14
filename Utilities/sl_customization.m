function sl_customization(cm)
% Change the order of libraries in the Simulink Library Browser. 

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

cm.LibraryBrowserCustomizer.applyOrder({'Space Robotics DQ',-2});
end