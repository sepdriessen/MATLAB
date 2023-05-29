function [I,cm,r] = MoI_2D_line(type,x,m)
% [I,cm,r] = MoI2D(type,x,m) calculates the moment of inertia I, center of
% mass distance cm and radius of gyration r of a 2D body that consists of
% multipe geometrical shapes positioned along a line, as defined by type,
% with corresponding parameters x and masses m. I is expressed from the
% center of mass.
%
% type is a cell array describing the body types (shapes).
% x is a cell array containing body parameters per body.
% m is an array describing the body masses.
%
% 'point': point mass, I=0
%    x = [distance];
% 'rod': slender beam, evenly distributed mass, I=m*l^2/12
%    x = [distance start, distance end];
% 'gyr': mass with radius of gyration (rog), I=m*rog^2
%    x = [distance,rog];
% 'wedge': slender beam with linearly changing cross section
%    x = [distance start, distance end, ratio surface start/end]
%    note: x = [a,b,Inf] returns NaN & should be implemented as x = [b,a,0]
%    note: x = [a,b,1] is the equivalent of a rod with x=[a,b]
%    note: x = [0,R,0] is the equivalent of a disc with radius R
% TO BE IMPLENTED
% 'cone': slender beam with quadratically changing cross section
%    x = [distance start, distance end, ratio surface start/end]
%    note: x = [0,R,0] is the equivalent of a sphere with radius R

if size(m,2)>size(m,1)
    m=m';
end

% calculate moment of inertia per body, about their own centre of mass
n=length(type);
Ii=zeros(n,1);
cmi=zeros(n,1);
for i1=1:n
    if strcmp(type{i1},'point')  % at radius x(1)
        Ii(i1)=0;
        cmi(i1)=x{i1}(1);
    elseif strcmp(type{i1},'rod') % between radius x(1) and x(2)
        Ii(i1)=(x{i1}(2)-x{i1}(1))^2/12*m(i1);
        cmi(i1)=mean(x{i1});
    elseif strcmp(type{i1},'gyr') % at radius x(1) with rog x(2)
        Ii(i1)=m(i1)*x{i1}(2)^2;
        cmi(i1)=x{i1}(1);
    elseif strcmp(type{i1},'wedge')
        r=x{i1}(3); % ratio         
        x2=x{i1}(2);
        x1=x{i1}(1);
        cmi(i1) =    (r*x2^3 + r*2*x1^3 - r*3*x2*x1^2 ...
                     - 3*x2^2*x1 + x1^3 + 2*x2^3)/(3*(r+1)*(x2-x1)^2);
        Ii(i1) = m(i1) * ((r*x2^4 + r*3*x1^4 - r*4*x2*x1^3 ...
                     - 4*x2^3*x1 + x1^4 + 3*x2^4)/(6*(r+1)*(x2-x1)^2) ...
                     - cmi(i1)^2);
    else
        warning('MATLAB:incorrectType',...
        ['Type ' num2str(i1) ' not recognized; entry ignored.'])
    end
end

% calculate moment of inertia about global centre of mass by using the
% parallel axis theorem
cm = m'*cmi/(ones(1,n)*m); % center of mass
I=ones(1,n)*Ii+m'*(cm-cmi).^2; % moment of inertia
r=sqrt(I/(ones(1,n)*m)); % radius of gyration