function Dyn = aux_setDynamics( dynamics )
% function Dyn = aux_setDynamics( dynamics )
%
%   Author1     :
% 
%   Date        : Winter 2018
%
%   Description : Create a Struct containing the system dynamics of the 
%                 model indicated by dynamics (the model must comply with 
%                 the naming convection
%
%   Parameters  : dynamics -> Name of the system model
% 
%   Return      : Dyn -> Struct containing the system dynamics
% 
%-------------------------------------------------------------------------%
import casadi.*




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          Set model parameters                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cmd_loadParameters  = ['parMdl = ' dynamics '_par();']; 
eval(cmd_loadParameters);    % Load parameters
switch nargout([dynamics '_dynamics'])
    case -4
        cmd_querySystemSize = ['[~,~,~,Dyn] = ' dynamics '_dynamics();'];
        eval(cmd_querySystemSize);   % Query sizes of dynamic system
    case -3
        cmd_querySystemSize = ['[~,~,Dyn] = ' dynamics '_dynamics();'];
        eval(cmd_querySystemSize);   % Query sizes of dynamic system
end

%-------------------------------------------------------------------------%




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          Define dynamic system                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define symbolic variables in SI units
x   = MX.sym('x', Dyn.Size.n_states); 
u   = MX.sym('u', Dyn.Size.n_inputs);


if Dyn.Size.n_onlineParameter > 0
    par = MX.sym('onlineParameter', Dyn.Size.n_onlineParameter);
    for ii = 1:Dyn.Size.n_onlineParameter
        fieldName = strsplit(Dyn.Info.onlineParameter{ii});
        parMdl.(fieldName{1}) = par(ii);
    end
end


switch Dyn.type
    case 'dae'
        % Continuous: xdot = f(x, p, u), y = g(x, p, u) and h(x, p, u) = 0
        % Discrete: xk1 = f(xk, pk, uk), yk = g(xk, pk, uk) and h(xk, pk, uk) = 0
        
        % Define algebraic variables in SI units
        p   = MX.sym('p', Dyn.Size.n_algebraicVars);
        
        % Get system dynamics
        eval(['[f, g, h] = ' dynamics '_dynamics(x, p, u, parMdl);']);
        
        % Define functions and store in struct
        if Dyn.Size.n_onlineParameter > 0
            Dyn.systemDynamics ...
                = Function([Dyn.name '_systemDynamics'], ...
                  {    x   ,     p    ,     u   ,      par    }, {      f         }, ...
                  {'states', 'algVars', 'inputs', 'parameters'}, {'systemDynamics'});

            Dyn.systemOutput ...
                = Function([Dyn.name '_systemOutput'], ...
                  {    x   ,     p    ,     u   ,      par    }, {     g        }, ...
                  {'states', 'algVars', 'inputs', 'parameters'}, {'systemOutput'});

            Dyn.algebraicEquation ...
                = Function([Dyn.name '_algebraicEquation'], ...
                {    x   ,     p    ,     u   ,      par    }, {        h          }, ...
                {'states', 'algVars', 'inputs', 'parameters'}, {'algebraicEquation'});

        else
            Dyn.systemDynamics ...
                = Function([Dyn.name '_systemDynamics'], ...
                  {    x   ,     p    ,     u   }, {      f         }, ...
                  {'states', 'algVars', 'inputs'}, {'systemDynamics'});

            Dyn.systemOutput ...
                = Function([Dyn.name '_systemOutput'], ...
                  {    x   ,     p    ,     u   }, {      g       }, ...
                  {'states', 'algVars', 'inputs'}, {'systemOutput'});

            Dyn.algebraicEquation ...
                = Function([Dyn.name '_algebraicEquation'], ...
                  {    x   ,    p    ,    u   }, {        h          }, ...
                  {'states','algVars','inputs'}, {'algebraicEquation'});

        end
       
        
    case 'ode'
        % Continuous: xdot = f(x, u), y = g(x, u) and h(x, u) = 0
        % Discrete: xk1 = f(xk, uk), yk = g(xk, uk) and h(xk, uk) = 0
        
        % Get system dynamics
        eval(['[f, g] = ' dynamics '_dynamics(x, u, parMdl);']);
        
        % Define functions and store in struct
        if Dyn.Size.n_onlineParameter > 0
            Dyn.systemDynamics ...
                = Function([Dyn.name '_systemDynamics'], ...
                  {    x   ,     u   ,      par    }, {      f         }, ...
                  {'states', 'inputs', 'parameters'}, {'systemDynamics'});

            Dyn.systemOutput ...
                = Function([Dyn.name '_systemOutput'], ...
                  {    x   ,     u   ,      par    }, {      g       }, ...
                  {'states', 'inputs', 'parameters'}, {'systemOutput'});

        else
            Dyn.systemDynamics ...
                = Function([Dyn.name '_systemDynamics'], ...
                  {    x   ,     u   }, {       f        }, ...
                  {'states', 'inputs'}, {'systemDynamics'});

            Dyn.systemOutput ...
                = Function([Dyn.name '_systemOutput'], ...
                  {    x   ,    u   }, {       g      }, ...
                  {'states','inputs'}, {'systemOutput'});

        end

end

disp('System dynamics set');

%-------------------------------------------------------------------------%