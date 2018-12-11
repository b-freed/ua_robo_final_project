function [total_dist, total_step] = simulate_walker_terrain_stoc_BOA_PD(T,controller,ifplot)

    %   Simulates an active walking robot from time 0 to T
    %   controller is a function with signature F = controller(t,y) that
    %   takes current time (t) and current state (y) and returns a forcing
    %   for the control of the active walker
    
    %   Code originally made by Mariano Garcia Anindya Chatterjee
    %   Modified by Ben Freed, Tianyu Wang, and Shuoqi Chen

    %   Based on:
    %   
    %   [1] Arthur D. Kuo "Energetics of Actively Powered Locomotion Using the
    %   Simplest Walking Model," ASME Journal of Biomedical Engineering, Vol. 124,
    %   No. 1, pp. 113-120, 2002. http://dx.doi.org/10.1115/1.1427703
    %   
    %   [2] M. Garcia, A. Chatterjee, A. Ruina, and M. Coleman, "The Simplest
    %   Walking Model: Stability, Complexity, and Scaling," ASME Journal of
    %   Biomedical Engineering, Vol. 120, No. 2, pp. 281-288, 1998.
    %   http://dx.doi.org/10.1115/1.2798313
    %   
    %   [3] Mario W. Gomes "A Derivation of the Transisition Rule at Heelstrike
    %   which appears in the paper 'The Simplest Walking Model: Stability,
    %   Complexity, and Scaling' by Garcia et al." pp. 1-3, Oct. 4, 1999.
    %   http://ruina.tam.cornell.edu/research/topics/locomotion_and_robotics/simplest_walking/simplest_walking_gomes.pdf

    %   Andrew D. Horchler, horchler @ gmail . com, Created 7-7-04
    %   Revision: 1.1, 5-1-16
    
    global event_counter 
    global gam
    global alpha
    global first_step
    global epsilon
    
    % for calculating BOA
    global theta_start
    global theta_dot_start
    global phi_start
    global phi_dot_start
    
    % first_step is to make sure the first step of the walker is not
    % influence by terrain stachasticity
    first_step = true;
    
    % event counter is to check the number of collision event
    event_counter = 0;

    % Gamma: angle of slope (radians), used by integration function
    gam = 0;  % gam = 0  means it's walking on the flat ground
    

    % Step period (seconds), frequency of sinusoidal forcing at hip
    tau = 3.84;

    % Magnitude of sinusoidal forcing at hip
    a = 0;

    % Spring constant of spring-like torque at hip
    %k = -0.08;
    
    % Integration time parameter
    per = 5;        % Max number of seconds allowed per step

    % Initial desired step length
    s = 0.4;

    % IC constants
    alpha = asin(0.5*s);
    

    % Toe-off impulse applied at heelstrike condition
    if a == 0
        omega = -1.04*alpha;
        P = -omega*tan(alpha);
    else
        et = exp(tau)+exp(-tau);
        omega = -alpha*(2+et)/et;
        c2aet = cos(2*alpha)*et;
        P = (c2aet*alpha-2*omega+c2aet*omega)/(2*sin(2*alpha));
    end

    % Calculate stable ICs from theoretically determined equations
%     y0 = [alpha;
%           omega;
%           2*alpha;
%           (1-cos(2*alpha))*omega];
    
    % Change to customized y0;
    y0 = [alpha;
          theta_dot_start;
          2*alpha;
          phi_dot_start];

    % Initialization
    y = [];         % Vector to save states
    t = [];         % Vector to save times
    tci = 0;        % Collision index vector
    h = [0 per];	% Integration period in seconds

    % Loop to perform integration of a noncontinuous function
    tf = 0;
    
    % let the walker walk unless the time is up
    while tf <= T
       
        
        % ==================================
        % Stochasitcity - bumpy terrain
        % ==================================
       
%         if first_step == false
%             okay_to_send = false;
%             while okay_to_send == false
%                 mu = 0;
%                 sigma = 1;
%                 epsilon = normrnd(mu, sigma);
%                 
%                 % send out the epsilon when it's smaller than 0.1 in
%                 % magnitude
%                 if abs(epsilon) <= 0.01
%                     okay_to_send = true;
%                 end
%                 
%             end
%         else
%             first_step = false;
%             epsilon = 0;
%         end
        
        epsilon = 0;
        
%         fprintf('epsilon: %f \n', epsilon);
        
        % ==================================
        % ==================================        
        
        colli = @(t,y) collision(t,y,epsilon);
   
        % Set integration tolerances, turn on collision detection, add more output points
        opts = odeset('RelTol',1e-4,'AbsTol',1e-8,'Refine',30,'Events',colli);
    
        % event_counter is to check if the number of collision event is
        % equal to the number of steps taken by the walker
        % If those numbers don't match, that means the walker has collide
        % with the ground multiple times before talking an meaningful step
        event_counter = event_counter+1;
        
       [tout,yout] = ode45(@(t,y) f(t,y,controller(t,y)),h,y0,opts); % Integrate for one stride
       y = [y;yout];                                         	%#ok<AGROW> % Append states to state vector
       t = [t;tout];                                            %#ok<AGROW> % Append times to time vector
       tf = t(end);
       c2y1 = cos(2*y(end,1));                               	% Calculate once for new ICs
       s2y1p = sin(2*y(end,1))*P;                               % Calculate once for new ICs
       
       y0 = [-y(end,1);
             c2y1*y(end,2)+s2y1p;
             -2*y(end,1);
             c2y1*(1-c2y1)*y(end,2)+(1-c2y1)*s2y1p];            % Mapping to calculate new ICs after collision
         
       tci = [tci length(t)];                                   %#ok<AGROW> % Append collision index to collision vector
       h = tf+[0 per];                                   	% New time step
       s = [s 2*sin(y0(1))];                                 	%#ok<AGROW> % Append last stride length to stride vector
       
       
    end
    
    %truncate these sequences down to the time span we want
    y = y(t <= T,:);
    t = t(t <= T);
    tci = tci(tci <= length(t));
    
    if ifplot

        % Graph collision map
        figure(1)
        plot(t,y(:,3)-2*y(:,1))
        grid on
        xlabel('time ( sqrt(l/g) )')
        ylabel('\phi(t)-2\theta(t) (rad.)')

        % Graph angular positions - the stride function
        figure(2)
        hold on
        plot(t,y(:,1),'r',t,y(:,3),'b--')
        grid on
        title('Stride Function')
        xlabel('time ( sqrt(l/g) )')
        ylabel('\phi(t), \theta(t) (rad.)')

        % Graph angular velocities
        figure(3)
        hold on
        plot(t,y(:,2),'r',t,y(:,4),'b--')
        grid on
        title('Angular Velocities')
        xlabel('time ( sqrt(l/g) )')
        ylabel('\phi^.(t), \theta^.(t) (rad./sqrt(l/g))')

        % Phase plot of phi versus theta
        figure(4)
        plot(y(:,1),y(:,3))
        grid on
        title('Phase Portrait')
        xlabel('\theta(t) (rad.)')
        ylabel('\phi(t) (rad.)')

        % Plot Hamiltonian
        H = 0.5*y(:,2).*y(:,2)+cos(y(:,1)-gam);
        figure(5)
        plot(t,H-H(1))
        grid on
        title('Hamiltonian - Total Energy of System')
        xlabel('time ( sqrt(l/g) )')
        ylabel('Hamiltonian: H(t)-H(0)')
    end

    % Run model animation: mview.m
    % y records the states
    % gam is angle of slope
    % tci is Collision index vector
    
    [total_step, total_dist] = wmview_BOA_PD(y,gam,tci,ifplot);

end


function ydot=f(t,y,F)
%ODE definition
% y1: theta
% y2: thetadot
% y3: phi
% y4: phidot
% F = forcing/control effort



gam = 0; %hardcoding this in for now

% First order differential equations for Simplest Walking Model
ydot = [y(2);
        sin(y(1)-gam);
        y(4);
        sin(y(1)-gam)+sin(y(3))*(y(2)*y(2)-cos(y(1)-gam))+F];

end


function [val,ist,dir]=collision(t,y, epsilon) %#ok<INUSL>

global flag
global event_counter 
global gam
global alpha
global first_step

%% Distance Guard Equation

stance_foot_x = 0;
stance_foot_y = 0;
hip_position_x = stance_foot_x - 1 * sin(y(1) - gam);          	% Position of hip
hip_position_y = stance_foot_y + 1 * cos(y(1) - gam);
swing_foot_position_x = hip_position_x - 1 * sin(y(3) - y(1) + gam);    	% Position of swing leg
swing_foot_position_y = hip_position_y - 1 * cos(y(3) - y(1) + gam);


swing_foot_position_y = swing_foot_position_y + epsilon;
val_detect = swing_foot_position_y - swing_foot_position_x * tan(gam);


if abs(val_detect)< 0.05
    if abs(y(3)) < 1.5*alpha        % Mannually filter out the zero appearing at vertical position
        val = 777; % just give val a positive value, no matter what it is
    else
        val = val_detect;
    end
else
    val = val_detect;
end

ist = 1;			% Stop integrating if collision found
dir = -1;			% Condition only true when passing from + to -

end
