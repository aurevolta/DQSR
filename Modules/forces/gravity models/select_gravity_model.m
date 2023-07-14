function gravity_fun_NAME = select_gravity_model(Gswitch)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

switch Gswitch
    case 1
        gravity_fun_NAME = uint8('NO_gravity');    
    case 2
        gravity_fun_NAME = uint8('ground_gravity_z');    
    case 3
        gravity_fun_NAME = uint8('gravity_model0');    
    case 4
        gravity_fun_NAME = uint8('gravity_model1');    
    case 5
        gravity_fun_NAME = uint8('gravity_modelCW');
end

end

