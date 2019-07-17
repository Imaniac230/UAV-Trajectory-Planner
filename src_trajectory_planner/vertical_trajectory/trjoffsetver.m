function [OutTraj] = trjoffsetver(InTraj,VOffset)
%TRJOFFSETVER - Apply a simple offset to the vertical trajectory
%
%   This function applies a specified offset [VOffset] to the vertical part of the input trajectory [InTraj].
%   Output is a matrix of geodetic (polar) coordinates with 3 columns representing [latitude longitude height]
%   respectively, with the offset applied to height.
%
%   [OutTraj] = TRJOFFSETVER(InTraj,VOffset)
%
%   The input trajectory [InTraj] must be a matrix of geodetic (polar) coordinates with 3 columns
%   [latitude longitude height] respectively. Offset [VOffset] must be a non-zero positive value given in meters [m].

%%
%inicializacne parametre %initial parameters
errTraj = 'Invalid input trajectory (1st parameter). Input must be a matrix of polar coordinates with 3 columns. For more info please visit help.';
errOfset = 'Invalid vertical offset (2nd parameter). Input must be a non-zero positive value. For more info please visit help.';
if ((size(InTraj,2) ~= 3) || ~isnumeric(InTraj))
    error(errTraj)
end
if (VOffset <= 0)
    error(errOfset)
end
OutTraj = InTraj;
%%
%aplikacia ofsetu %offset application
hAGL = InTraj(:,3) + VOffset;
hAGLFilt = designfilt('lowpassiir','FilterOrder',7,'HalfPowerFrequency',0.15,'DesignMethod','butter');
try
    OutTraj(:,3) = filtfilt(hAGLFilt, hAGL);
catch err
    message = ['Filter not used.' ' ' err.message];
    warning('OffsetAlgorithm:FilterNotUsed',message)
end
end