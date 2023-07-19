function D=Dm(a)
% This matrix differs from Sm for a single sign and is used only in the
% lagrange formulation. 

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% qmatrix
Qa=2*crossqm(a(1:4))';

D=[2*crossqpt(a(1:4)),zeros(4);2*crossqmt(a(5:8)),Qa];


end