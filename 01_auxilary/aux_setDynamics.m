function Dyn = aux_setDynamics( dynamics )
% function Dyn = aux_setDynamics( dynamics )
%
%   Author1    : 
% 
%   Date       :  Winter 2018
%
%   Description:  Create a Struct containing the system dynamics of the 
%                 model indicated by dynamics (the model must comply with 
%                 the naming convection
%
%   Parameters :  dynamics -> Name of the system model
% 
%   Return     :  Dyn -> Struct containing the system dynamics
% 
%   Examples of Usage: None



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
try
    cmd_querySystemSize = ['[~,~,~,Dyn] = ' dynamics '_dynamics();'];
    eval(cmd_querySystemSize);   % Query sizes of dynamic system
catch
    cmd_querySystemSize = ['[~,~,Dyn] = ' dynamics '_dynamics();'];
    eval(cmd_querySystemSize);   % Query sizes of dynamic system
end

%-------------------------------------------------------------------------%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          Define dynamic system                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: Add option to switch between casadi and matlab syms symbolic variables for states, algebraic variables and controls
% TODO: Online parameter necessary here? Rather mpc specific

% SI-units
x         = MX.sym('x', Dyn.Size.n_systemStates); 
p         = MX.sym('p', Dyn.Size.n_algebraicVars);
u         = MX.sym('u', Dyn.Size.n_controlInputs);    
parOnline = MX.sym('parOnline', Dyn.Size.n_parOnline);

% Continuous: x_dot = f(x, p, u), y = g(x, p, u) and h(x, p, u) = 0
% Discrete: xk1 = f(xk, pk, uk), yk = g(xk, pk, uk) and h(xk, pk, uk) = 0
switch Dyn.type
    case 'dae'
        if Dyn.Size.n_parOnline~=0
            eval(['[f, g, h] = ' dynamics '_dynamics(x, p, u, parMdl, parOnline);']);
        else
            eval(['[f, g, h] = ' dynamics '_dynamics(x, p, u, parMdl);']);
        end
    case 'ode'
        if Dyn.Size.n_parOnline~=0
            eval(['[f, g] = ' dynamics '_dynamics(x, u, parMdl, parOnline);']);
        else
            eval(['[f, g] = ' dynamics '_dynamics(x, u, parMdl);']);
        end
end

% Define functions and store in struct
Dyn.systemDynamics ...
    = Function([Dyn.name '_systemDynamics'], ...
      { x ,  p ,  u ,  parOnline } , { f }, ...
      {'x', 'p', 'u', 'parOnline'} , {'systemDynamics'});

Dyn.systemOutput ...
    = Function([Dyn.name '_systemOutput'], ...
      { x,   p,   u,   parOnline } , { g }, ...
      {'x', 'p', 'u', 'parOnline'} , {'systemOutput'});

if strcmp(Dyn.type, 'dae')
    Dyn.algebraicEquation ...
        = Function([Dyn.name '_algebraicEquation'], ...
        { x,   p,   u,   parOnline } , { h }, ...
        {'x', 'p', 'u', 'parOnline'} , {'algebraicEquation'});
end

disp('dynamics set');

%-------------------------------------------------------------------------%