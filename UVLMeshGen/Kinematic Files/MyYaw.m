function [Alpha, DAlpha] = MyYaw (TimeSteps, DT, Yaw0)

% TimeSteps --> Number of time steps (Input argument)
% DT --> Time steps (Input argument)
% Yaw0 --> Initial Yaw angle [º] (Input argument)

Yaw0 = Yaw0 * pi / 180;

Yaw_Amp = 60 * pi / 180;
Yaw_W   = 0.35;

TIME = linspace (0, TimeSteps*DT, TimeSteps+1);

Alpha(1:TimeSteps+1)  = Yaw0 + Yaw_Amp * sin (Yaw_W * TIME(1:TimeSteps+1) );
DAlpha(1:TimeSteps+1) = Yaw_Amp * Yaw_W * cos (Yaw_W * TIME(1:TimeSteps+1) );

end
