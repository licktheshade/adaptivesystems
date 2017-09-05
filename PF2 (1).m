function [] = PF ()
% Just call PF (without any arguments) to run the animation
% 
% This is the matlab code behind the movie "Particle Filter Explained
% without Equations", which can be found at http://youtu.be/aUkBa1zMKv4
% Written by Andreas Svensson, October 2013
% Updated by Andreas Svensson, February 2013, fixing a coding error in the
% 'propagation-update' of the weights
% andreas.svensson@it.uu.se
% http://www.it.uu.se/katalog/andsv164
% 
% The code is provided as is, and I take no responsibility for what this
% code may do to you, your computer or someone else.
%
% This code is licensed under a
% Creative Commons Attribution-ShareAlike 3.0 Unported License.
% http://creativecommons.org/licenses/by-sa/3.0/

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Setup and initialization %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Setting the random seed, so the same example can be run several times
s = RandStream('mt19937ar','Seed',1);
RandStream.setGlobalStream(s);
pauseT = 0.25;

% Some unceratinty parameters
measurementNoiseStdev = 0.1;
observNoiseStdev = 0.1; 
speedStdev = 0.25;

% Speed of the aircraft
speed = 2;
% Set starting position of aircraft
planePosX = 2; 
planePosY = 10;

% Some parameters for plotting the particles
m = 1000; k = 0.0001;

% Number of particles
N = 200;

% Some variables for plotting
bX = 0;
tX = 40;
plotVectorMountains = [bX:0.01:tX];

plot2 = plotVectorMountains;
plotHeight = 17;

% The function describing the ground
ground = @(x)  (sin(pi*x))/10+(x*3)/12;

%%Problem 9
%%sin(x)+sin(x*0.66);
%%
%%Simple Sin Wave
%% sin(x)
%%
%Other Unused Functions
%%%%2*cos(x)+cos(2*x);
%%-exp(-x)*sin(2*pi*x);

%%Differentiation of ground function
diffground = @(x) pi*cos(pi*x)/10+1/4;

%%Problem 9
%0.66*cos(0.66*x)+cos(x);
%
%%Simple Sin Wave
%%cos(x);
%
%%Other Unused Functions
%%(-3*cos(x))*(sin(x).^2)-(3*exp(-3*x));
%%-2*(sin(2*x)+sin(x));
%%-sin(x)-((x)*cos(x));

% Plot the environment
area(plotVectorMountains,ground(plotVectorMountains),'FaceColor',[0 0.6 0])
set(gca,'XTick',[]); set(gca,'YTick',[]); hold on
axis([bX tX 0 20])
plane = plotPlane(planePosX,planePosY,1);
measurementLine = line([planePosX planePosX],[ground(planePosX),planePosY],'Color',[1 0 0],'LineStyle',':');
pause(1)


%%%%%%%%%%%%%%%%%%%%%%%
%%% Begin filtering %%%
%%%%%%%%%%%%%%%%%%%%%%%

%%Kalman%%%

%%Iterations
dur = 50;

%Initialse values, noise and covariance values
X = planePosX+V;
P = 1;
W = randn*measurementNoiseStdev;
V = randn*observNoiseStdev;
speedNoise = speedStdev*randn;
Q = cov(W);
R = cov(V);

%Residual covariance
S = 0;

%%measurement value Z
planeMeasDist = (planePosY - ground(planePosX))+W;


%%time step
k = 0;

for t = 1:dur;

    %Prediction
    
    X_1 = X;

    X = X_1+speed+speedNoise;


    %Unecessary Jacobian Fx = 1;
    
    %Observation Jacobian
    Hx = diffground(X);
    P=P+Q;
    
    %%Z

    Obs2 =  planePosY - ground(X)+randn*measurementNoiseStdev;
    
    %%Update
    
    %%Measurement residual
    y = planeMeasDist-ground(X);
    %%Residual covariance
    S_1 = S ;
    S = (Hx*P*Hx)+R;    
    %%KalGain = P/(P+R)
    K = Hx*P*S_1;
    X_1 = X;
    X = X_1+(K*y);
    

    %%Updated covariance estimate
    P = (1-(K*Hx))*P;
    k = k+1;
    
    %Plane propogation and plot update
    
    planePosX = planePosX + speed;

    
    delete(plane); 
    delete(measurementLine);
    plane = plotPlane(planePosX,planePosY,1);

    %Utilising the covariance to plot esstimation confidence
    circsize = 5*(abs(sqrt(P)));
    
    plot(X,ground(X)+Obs2,'o','Color','k','MarkerSize',circsize)
    
    %Line from plane to estimated ground position
    measurementLine = line([planePosX planePosX],[ground(X),planePosY],'Color',[1 0 0],'LineStyle',':');
    pause(pauseT);
end
end
% 
% % %%%Particle%%%%
% % % 
% 
% % Generate particles
% particles = rand(N,1)*tX+bX;
% 
% % Plot particles
% particleHandle = scatter(particles,plotHeight(ones(size(particles))),m*(1/N*ones(N,1)+k),'k','filled');
% pause(1)
% 
% FirstRun = 1;
% 
% % Initialize particle weights
% w = 1/N*ones(N,1);
% 
% for t = 1:60
%     % Generate height measurements (with gaussian measurement noise)
%     planeMeasDist = planePosY - ground(planePosX) + randn*measurementNoiseStdev;
%     
%     % Evaluate measurements (i.e., create weights) using the pdf for the normal distribution
%     w = w.*(1/(sqrt(2*pi)*measurementNoiseStdev)*exp(-((planePosY-ground(particles))-planeMeasDist).^2/(2*measurementNoiseStdev^2)));
%     
%     % Normalize particle weigths
%     w = w/sum(w);
% 
%     if FirstRun
%         
%         % Sort out some particles to evaluate them "in public" the first
%         % run (as in the movie)
%         [~, order] = sort(w,'descend');
%         pmax = order(1);
%         pmaxi = setdiff(1:N,pmax);
%         delete(particleHandle)
%         particleHandle = scatter([particles(pmaxi);particles(pmax)],plotHeight(ones(size(particles))),m*([ones(N-1,1)/N;w(pmax)]+k),'k','filled');
%         pause(1)
%         
%         pmax2 = order(2);
%         pmaxi2 = setdiff(pmaxi,pmax2);
%         delete(particleHandle)
%         particleHandle = scatter([particles(pmaxi2);particles(pmax);particles(pmax2)],plotHeight(ones(size(particles))),m*([ones(N-2,1)/N;w(pmax);w(pmax2)]+k),'k','filled');
%         pause(1)
%         
%         % Plot all weighted particles    
%         delete(particleHandle)
%         particleHandle = scatter(particles,plotHeight(ones(size(particles))),m*(w+k),'k','filled');
%         pause(1)
%     end
% 
%     % Resample the particles
%     u = rand(N,1); 
%     wc = cumsum(w);
%     [~,ind1] = sort([u;wc]); 
%     ind=find(ind1<=N)-(0:N-1)';
%     particles=particles(ind,:); 
%     w=ones(N,1)./N;
% 
%     delete(particleHandle);
%     particleHandle = scatter(particles,plotHeight(ones(size(particles))),m*(w+k),'k','filled');
%     pause(0.25)
% 
%     % Time propagation
%     speedNoise = speedStdev*randn(size(particles));
%     particles = particles + speed + speedNoise;
%     
%     % Update weights
%     % w = w, since the update in the previous step is done using our motion model, so the
%     % information is already contained in that update.
%     
%     % Move and plot moved aircraft
%     planePosX = planePosX + speed;
%     delete(plane); delete(measurementLine)
%     plane = plotPlane(planePosX,planePosY,1);
%     measurementLine = line([planePosX planePosX],[ground(planePosX),planePosY],'Color',[1 0 0],'LineStyle',':');
%     
%     if FirstRun
%         % Plot updated particles
%         delete(particleHandle)
%         particleHandle = scatter(particles,plotHeight(ones(size(particles))),m*(w+k),'k','filled');
%         pause(1)
%     end
% 
%     FirstRun = 0;
% end
% end
% % 
function [ h ] = plotPlane( xpos,ypos,fignr )
figure(fignr)

X = xpos - 0.6 + [-1,     -0.1,   -0.09,    0.3,  0.7, 0.8, 0.7, 0.3, -0.09,  -0.1, -1];
Y = ypos + [-0.05, -0.05, -0.4, -0.05, -0.05,0 0.05, 0.05, 0.4, 0.05, 0.05];
h = fill(X,Y,'k');
end
