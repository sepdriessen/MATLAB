function y=filly(y,ylim,ylimrange,yrange)
% function y=filly(y,ylim,ylimrange,yrange)
% scale y such that y covers ylim.
% optional:
% ylimrange
%   range of ylim to be covered
%   default ylimrange=[0 1] (=[bottom,top])
% yrange:
%   range of y to be covered within ylim
%   default minmax(y)

if nargin<4
    yrange=minmax(y')';
    if nargin<2
        ylim=get(gca,'YLim');
    end
end
if nargin>2
    ylim = ylim(1) + diff(ylim).*ylimrange;
end
y = ylim(1) + (y-yrange(1))*diff(ylim)/diff(yrange);