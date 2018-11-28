function aux_plotSize( width, height, position_x, position_y, labelWidth_x, labelWidth_y, whiteSpace_top, whiteSpace_right)
% function aux_plotSize( width, height, position_x, position_y, labelWidth_x, labelWidth_y, whiteSpace_top, whiteSpace_right)
%
%   Author1     : 
% 
%   Date        : Winter 2018
%
%   Description : This function creates a plot with a fixed font of 8 pt. 
%                 The size of the plot can be set by specifying x and y. 
%                 The position of the plot on the screen is then set by
%                 xx and yy. All measurements are in centimeters.
%
%   Parameters  : width ->
%                 height ->
%                 position_x ->
% 
%   Return      : None
%
%-------------------------------------------------------------------------%
switch nargin
    case 4
        % Define white space necessary for labeling
        labelWidth_x = 1.2 ; % cm 
        labelWidth_y = 1.2 ; % cm 
        
        % Define white space on top and right side of figure
        whiteSpace_top = 0.8 ; % cm
        whiteSpace_right = 0.8 ; % cm
        
    case 6
        % Define white space on top and right side of figure
        whiteSpace_top = 0.8 ; % cm
        whiteSpace_right = 0.8 ; % cm
        
end


% Calculate lower left corner coordinate of figure
LL_corner_x = labelWidth_x/width ; 
LL_corner_y = labelWidth_y/height ;


% Calculate figure width
Width_x = (width - labelWidth_x-whiteSpace_right)/width ;
Width_y = (height - labelWidth_y-whiteSpace_top)/height ;


% Define axes
axes('position',[LL_corner_x LL_corner_y Width_x Width_y]) ;


% Change color
% set(gcf,'Color',[1 1 1]) % Set white background color


% Change absolute figure size

set(gcf,'PaperUnits','centimeters') 
set(gcf,'Units','centimeters')
xSize = width ; ySize = height ; xpos = position_x ; ypos = position_y ; 
xLeft = (21-xSize)/2; yTop = (29.7-ySize)/2 ;  
set(gcf,'PaperPosition',[xLeft yTop xSize ySize])
set(gcf,'PaperPositionMode','Auto')
set(gcf,'Position',[xpos ypos xSize ySize])

set(gcf,'PaperSize',[xSize ySize]);