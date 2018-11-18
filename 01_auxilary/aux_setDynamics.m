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



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                             Import toolboxes                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
import casadi.*

%-------------------------------------------------------------------------%



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
% TODO: Add option to switch between casadi and matlab syms symbolic variables for states, algebraic variables and controls
% TODO: Online parameter necessary here? Rather mpc specific

% Continuous: x_dot = f(x, p, u), y = g(x, p, u) and h(x, p, u) = 0
% Discrete: xk1 = f(xk, pk, uk), yk = g(xk, pk, uk) and h(xk, pk, uk) = 0
switch Dyn.type
    case 'dae'
        % Define symbolic variables in SI units
        x   = MX.sym('x', Dyn.Size.n_states); 
        p   = MX.sym('p', Dyn.Size.n_algebraicVars);
        u   = MX.sym('u', Dyn.Size.n_inputs); 
        
        eval(['[f, g, h] = ' dynamics '_dynamics(x, p, u, parMdl);']);
        
        % Define functions and store in struct
        Dyn.systemDynamics ...
            = Function([Dyn.name '_systemDynamics'], ...
              { x      ,  u      ,  p       } , { f }, ...
              {'states', 'inputs', 'algVars'} , {'systemDynamics'});

        Dyn.systemOutput ...
            = Function([Dyn.name '_systemOutput'], ...
              { x      ,  u      ,  p       } , { g }, ...
              {'states', 'inputs', 'algVars'} , {'systemOutput'});

        Dyn.algebraicEquation ...
            = Function([Dyn.name '_algebraicEquation'], ...
              { x      ,  u      ,  p       } , { h }, ...
              {'states', 'inputs', 'algVars'} , {'algebraicEquation'});
          
    case 'ode'
        % Define symbolic variables in SI units
        x   = MX.sym('x', Dyn.Size.n_states); 
        u   = MX.sym('u', Dyn.Size.n_inputs); 
        
        eval(['[f, g] = ' dynamics '_dynamics(x, u, parMdl);']);
        
        % Define functions and store in struct
        Dyn.systemDynamics ...
            = Function([Dyn.name '_systemDynamics'], ...
              { x      ,  u      } , { f }, ...
              {'states', 'inputs'} , {'systemDynamics'});

        Dyn.systemOutput ...
            = Function([Dyn.name '_systemOutput'], ...
              { x      ,   u     } , { g }, ...
              {'states', 'inputs'} , {'systemOutput'});
          
end

sprintf('Nonlinear dynamics set\n');

%-------------------------------------------------------------------------%