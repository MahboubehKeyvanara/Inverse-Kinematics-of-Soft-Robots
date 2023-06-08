function param = Inverse_Kinematics_final(P_d,L_link,numseg,num_link,plotting)
%%%%%%%%%%%%%%%%
% This function calculates the desired degrees of freedom for the
% extensible PCC soft robot. The inputs of this function are:
% P_d : desired path
% L : the length of each soft robot segment
% numseg: the number of robot segment
% num_link: the number of link in each segment 
% Plotting : True for activate plotting (False by default) Plotting better
% be deactivated for computation cost reasons.
% 

%outputs: 
%param.L = the length of each segment
%param.th = bending of each segment
% param.phi = phi of each segment
% param.err = err of each segment 

%%%%%%%%%%%%%%%%

%% checking if the given data 
if ~isa(P_d,"double") || ~isa(L_link,"double")|| ~isa(numseg,"double")|| ~isa(num_link,"double") 
    error("Please Enter correct input")
end


%% initialization

d_L = zeros(numseg,size(P_d,2)); %changes in length of each segment
th = zeros(numseg,size(P_d,2)); %bending in each segment 
phi= zeros(numseg,size(P_d,2)); % phi of each segment 
P=zeros(3,num_link+1,numseg);
effector = zeros(3,size(P_d,2));
% x0 = zeros(numseg,size(P_d,2));

% Boundary Conditions
lb(1:3:3*numseg) = -(L_link/10); % Lower boundary of length
lb(2:3:3*numseg) = -140/(2*num_link); % lower boundary of bending angle
lb(3:3:3*numseg) = -180; % Lower Boundary of Phi
ub(1:3:3*numseg) = (L_link/10); % Higher boundary of length
ub(2:3:3*numseg) = 140/(2*num_link); % Higher boundary of bending angle
ub(3:3:3*numseg) = 180; % Higher boundary of Phi

% Figure Initialize
figure('units','pixels','position',[0 100 1920 1080])
hold on
% Uncomment next three lines if video recording is required
% v = VideoWriter('3Segment_4tu.avi');
% v.FrameRate = 10;
% open(v)


%% solution using fmincon
for k = 2:size(P_d,2)
    
    AllEqn = @(S)(forward_kinematics(S,num_link,numseg,L_link)-P_d(:,k)); %The optimization equation
   
    %Fmincon Boundary condition
    A = []; b = []; 
    Aeq = []; beq = [];
%     x0 = [d_L(1,k-1);th(1,k-1);phi(1,k-1);d_L(2,k-1);th(2,k-1);phi(2,k-1);d_L(3,k-1);th(3,k-1);phi(3,k-1)];
    x0 = [];
    for numseg_iter = 1:numseg
        x0 = [x0; d_L(numseg_iter,k-1);th(numseg_iter,k-1);phi(numseg_iter,k-1)];
    end

    options = optimoptions(@fmincon,'Algorithm','sqp'); % setting up the fmincon
    sol_fmin = fmincon(@(S)norm(AllEqn(S)),x0,A,b,Aeq,beq,lb,ub,[],options); % Solving using fmincon
    % The calculated Parameters for each segment
    d_L(:,k) = sol_fmin(1:3:end);
    th(:,k) = sol_fmin(2:3:end);
    phi(:,k) = sol_fmin(3:3:end);

    % solving the problem with sudden changes from phi at -180 to 180 or
    % vice versa
    for seg = 1:numseg
        if phi(seg,k)==180 || phi(seg,k)==-180
            phi(seg,k)= -phi(seg,k);
        end      
    end

    %plotting the desired path and calculated inverse kinematics 
    
     if nargin>4 && plotting == true % Checking if the plot is activated
       % calculating the postion of each link (can be modified so uses the
       % forward kinematics function directly)
       
       
         for seg = 1:numseg 
         alpha = eye(3);
         if seg>1
            P(:,1,seg) = P(:,end,seg-1);
            for rotation=1:seg-1
                alpha = alpha*rotz(phi(rotation,k))*roty(2*num_link*th(rotation,k));
            end
         end
        for i =2:num_link+1

            P(:,i,seg) = P(:,i-1,seg)+(L_link+d_L(seg,k))*alpha...
                *[cosd(phi(seg,k))*sind((2*i-3)*th(seg,k)); sind(phi(seg,k))*sind((2*i-3)*th(seg,k)); cosd((2*i-3)*th(seg,k))];
            plot(0,0)
            hold on
        end
         end
         %Plotting setting can be modified as needed
        set(gca,'fontsize',24,'FontName','Arial','linewidth',3)
        fontsize=24;
        ax = gca;
        ax.GridAlpha = 0.3;
        ax.XAxis.LineWidth = 3;
        ax.XAxis.FontSize =fontsize;
        ax.YAxis.LineWidth = 3;
        ax.YAxis.FontSize =fontsize;
        ax.YAxis.FontName ='Arial';
        for seg = 1:numseg
            pl{seg} = plot3(P(1,:,seg),P(2,:,seg),P(3,:,seg),'o-','LineWidth',4);
        end
         h = plot3(P_d(1,2:end),P_d(2,2:end),P_d(3,2:end),'-','Color',[0.5,0.5,0.5],'Linewidth',2);
%         pl{1}.Color = 'black';
%         pl{2}.Color = [0 0.4470 0.7410];
        xlabel('X (mm)','FontSize',fontsize)
        ylabel('Y (mm)','FontSize',fontsize)
        zlabel('Z (mm)','FontSize',fontsize)
        view(160,40)
        axis equal
        grid on
        effector(:,k) = P(:,end,end); 
        f = plot3(effector(1,2:k),effector(2,2:k),effector(3,2:k),'m-','LineWidth',4);
        legend([h f],'Desired Path','End-effector IK','Location','northeast')
        hold off
        err(k) = norm(P_d(:,k)-effector(:,k));
        pause(0.001)
        % Uncomment next 2 lines for video
    %     frame = getframe(gcf);
    %     writeVideo(v,frame)

     end
    
end
% Uncomment next line for the Video
% close (v)



for a = 1:length(th)
    for seg = 1:numseg
        if th(seg,a)<0
            if phi(seg,a)<0
                phi(seg,a) =  180+phi(seg,a);
                continue
            end
            if phi(seg,a)>0
                phi(seg,a) =  -180+phi(seg,a);
            end
        end
    end
end


%% Desired Parameters
param.L = num_link*(d_L(:,2:end)+L_link);
param.th = (2*num_link)*abs(th(:,2:end));
param.phi = phi(:,2:end);
if nargin>4
    param.err = err(2:end);
end
end
