close all; clear; clc; 

load data_vrep.mat

l_0 = 0.5;
l_occ = 0;
l_free = 1;

% normalize data
X(1,:) = round(X(1,:)*100,5);
X(2,:) = round(X(2,:)*100,5);
z(1,:) = round(z(1,:)*100,5);

% create space for map
xmn = min(X(1,:),[],'all') -500; 
xmx = max(X(1,:),[],'all') +500;
ymn = min(X(2,:),[],'all') -500; 
ymx = max(X(2,:),[],'all') +500;

% create grid
stepx = 1; stepy = 1;
x = [xmn:stepx:xmx]; 
y = [ymn:stepy:ymx];


% initialize grid value 1 grey, 2 black, 3 white
C = l_0*ones(length(y),length(x));


[XGrid, YGrid ] = meshgrid(x,y);
figure(1); clf; hold on             % Plot original data points                 % Plot original data points
h_grid = pcolor(XGrid,YGrid,C);
h_grid.EdgeColor='none';

% colorMap = [0.9 0.9 0.9; 0 0 0; 1 1 1]; % grey, black, white
colorMap = bone(20);
colormap(colorMap);

% handles for plots
h_robot = plot(0,0,'bo','MarkerSize',10, 'LineWidth',1);  %robot body 
h_robothead = quiver(0,0,0,0,'b','LineWidth',1); % real robot heading line
h_laser = plot(0,0,'g-.','LineWidth',1); % laser

axis equal;
axis([-50 550 -50 550]);

xt = get(gca, 'XTick');
set(gca, 'XTick',xt, 'XTickLabel',xt/100)
yt = get(gca, 'YTick');
set(gca, 'YTick',yt, 'YTickLabel',yt/100)

for tt=1:size(z,3)
    
    xr = X(1,tt);
    yr = X(2,tt);
    thr = X(3,tt);
    
    % plot
    h_robot.XData = xr; 
    h_robot.YData = yr; 
    h_robothead.XData = xr; h_robothead.YData = yr;
    h_robothead.UData = 5*cos(thr); 
    h_robothead.VData = 5*sin(thr);
    drawnow;
    
    x_laser = [];
    y_laser = [];
    B = [];
    
    for phi = 1:size(z,2)
        
        sen_dis = z(1,phi,tt); % distance
        sen_ang = z(2,phi,tt); % angle relative to robot
        
        irx = round((xr-xmn)/stepx);
        iry = round((yr-ymn)/stepy);
        
        % for laser plotting
       
        [x0,y0,x1,y1]= find_xy01(xr,yr,thr,sen_dis,sen_ang);
        x1 = round(x1,5);
        y1 = round(y1,5);
        x_laser = [x_laser x0 x1 NaN];
        y_laser = [y_laser y0 y1 NaN];
        
        [ix, iy] = bresenham(x0,y0,x1,y1);
        B_L = [ix, iy];
        x_offset = 459;
        y_offset = 390; %452
        
        for k = 1:4:size(B_L,1)
            for i=ix(k)
                for j=iy(k) 
                    if ismember([i,j],B_L(1:end-1,:),'rows') == 1
                        
                       if C(j+y_offset,i+x_offset) == l_0
                       C(j+y_offset,i+x_offset) = C(j+y_offset,i+x_offset) + l_free - C(j+y_offset,i+x_offset);%l_0;
                       end
                       
                    elseif ismember([i,j],B_L(end,:),'rows') == 1 
                       %if C(j+y_offset,i+x_offset) == l_0
                        C(j+y_offset,i+x_offset) = C(j+y_offset,i+x_offset) + l_occ - C(j+y_offset,i+x_offset);%l_0;
                       %end
                    end
                end        
            end
        end
        
        
    end
    
    h_laser.XData = x_laser;    h_laser.YData = y_laser;
%   pause(0.2);
    h_grid.CData = C;
%   pause(0.1);
    
    
    
end


function [x0,y0,x1,y1] = find_xy01(xr, yr, thr, sensor_dis, sensor_ang)
    x0 = xr; 
    y0 = yr;
    x1 = xr + sensor_dis*cos(thr+sensor_ang);
    y1 = yr + sensor_dis*sin(thr+sensor_ang);
end


function [x, y]=bresenham(x1,y1,x2,y2)
x1=round(x1); 
x2=round(x2);
y1=round(y1); 
y2=round(y2);
dx=abs(x2-x1);
dy=abs(y2-y1);
steep= abs(dy)>abs(dx);
if steep 
    t=dx;dx=dy;dy=t; 
end
if dy==0 
    q=zeros(dx+1,1);
else
    q=[0;diff(mod([floor(dx/2):-dy:-dy*dx+floor(dx/2)]',dx))>=0];
end
if steep
    if y1<=y2 
        y=[y1:y2]'; 
    else
        y=[y1:-1:y2]';
    end
    if x1<=x2 
        x=x1+cumsum(q);
    else
        x=x1-cumsum(q);
    end
else
    if x1<=x2 
        x=[x1:x2]'; 
    else
        x=[x1:-1:x2]';
    end
    if y1<=y2 
        y=y1+cumsum(q);
    else
        y=y1-cumsum(q); 
    end
end
end
