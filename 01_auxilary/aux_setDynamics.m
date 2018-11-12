function dyn = aux_setDynamics( dynamics )
% function dyn = aux_setDynamics( dynamics )
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
%   Return     :  dyn -> Struct containing the system dynamics
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
cmd_loadParameters  = ['par = ' dynamics '_par();']; 
cmd_querySystemSize = ['[~,~,~,dyn] = ' dynamics '_dynamics();'];
eval(cmd_loadParameters);    % Load parameters
eval(cmd_querySystemSize);    % Query sizes of DAE

%-------------------------------------------------------------------------%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                               Define DAE                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: Add option to switch between casadi and matlab syms symbolic variables for states, algebraic variables and controls

% SI-units
xk      = MX.sym('xk', dyn.size.n_systemStates); 
pk      = MX.sym('pk', dyn.size.n_algebraicVars);
uk      = MX.sym('uk', dyn.size.n_controlInputs);    
parOn   = MX.sym('parOn', dyn.size.n_parOn);

% Rhs x_k+1=f(x_k, p_k, u_k), y_k=g(x_k, p_k, u_k) and 0=h(x_k, p_k, u_k)
eval(['[xk1, yk, hk] = ' dynamics '_dynamics(xk, pk, uk, parOn, par);']);

% Define functions and store in struct
dyn.systemDynamics ...
    = Function([dyn.name '_systemDynamics'], ...
      { xk ,  pk ,  uk ,  parOn}  , { xk1}, ...
      {'xk', 'pk', 'uk', 'parOn'} , {'systemDynamics'});

dyn.systemOutput ...
    = Function([dyn.name '_systemOutput'], ...
      { xk,   pk,   uk,   parOn}  , { yk}, ...
      {'xk', 'pk', 'uk', 'parOn'} , {'systemOutput'});

dyn.algebraicEquation ...
    = Function([dyn.name '_algebraicEquation'], ...
      { xk,   pk,   uk,   parOn}  , { hk}, ...
      {'xk', 'pk', 'uk', 'parOn'} , {'algebraicEquation'});

disp('dae set');

%-------------------------------------------------------------------------%