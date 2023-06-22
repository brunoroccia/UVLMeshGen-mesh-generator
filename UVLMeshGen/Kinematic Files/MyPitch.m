function [Beta, DBeta] = MyPitch (TimeSteps, DT, Pitch0)

% TimeSteps --> Number of time steps (Input argument)
% DT --> Time steps (Input argument)
% Pitch0 --> N-vector containing the Initial Pitch angle [º] for each blade (Input argument)

Pitch0 = Pitch0 * pi / 180;

TIME = linspace (0, TimeSteps*DT, TimeSteps+1);

NN = length(Pitch0);

Beta = zeros (NN,TimeSteps+1);

for i = 1:NN
    Beta(i, :)  = Pitch0(i) * ones (1,TimeSteps+1);
end

DBeta = zeros (NN,TimeSteps+1);

end
