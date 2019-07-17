function [OutDist] = trjphotogr2linedist(FocLen,FlHeight,SenSize,OvrlP,FwdSenPar)
%TRJPHOTOGR2LINEDIST - Distance between trajectory lines
%
%   This function returns a distance value in meters [OutDist] between horizontal trajectory lines
%   based on the focal length of the used camera [FocLen], chosen height
%   above the terrain [FlHeight], matrix of camera size parameters [SenSize] and the desired 
%   side overlap percetage of photographs [OvrlP].
%
%   [OutDist] = TRJPHOTOGR2LINEDIST(FocLen,FlHeight,SenSize,OvrlP,FwdSenPar)
%
%   The focal length is given in milimeters [mm]. Flight height is given in meters [m]. The sensor size must
%   be an array of non-zero lengths ([a b] or [a;b]) given in milimeters [mm].
%   Overlap must be a percentage value from 0% to 100%. [FwdSenPar] specifies which camera size parameter is 
%   perpendicular to the flight path. It must be either 1 specifying sensor parameter "a",
%   or 2 specifying sensor parameter "b".

%%
%inicializacne parametre
%initial parameters
errFocal = 'Invalid focal length (1st parameter). Input must be a non-zero positive value. For more info please visit help.';
errHeight = 'Invalid flight height (2nd parameter). Input must be a non-zero positive value. For more info please visit help.';
errSensor = 'Invalid sensor size (3rd parameter). Input must be a vector of two non-zero positive values. For more info please visit help.';
errOvrl = 'Invalid overlap (4th parameter). Input must be a value from 0 to 100. For more info please visit help.';
errOrient = 'Invalid last parameter. Input must be eiher 1 or 2. For more info please visit help.';
errSwitch = 'Please verify that the focal length is not larger than the height above terrain. For more info please visit help.';
errFullFrame = 'The size of the sensor is larger than a full frame, please verify your sensor parameters.';
%%
%kontrola vstupnych parametrov %input parameters verification
if (~isnumeric(FocLen) || (sum(size(FocLen)) ~= 2))
    error(errFocal)
end
if (~isnumeric(FlHeight) || (sum(size(FlHeight)) ~= 2))
    error(errHeight)
end
if (~isnumeric(OvrlP) || (sum(size(OvrlP)) ~= 2))
    error(errOvrl)
end
if (~isnumeric(FwdSenPar) || (sum(size(FwdSenPar)) ~= 2))
    error(errOrient)
end
if (sum(size(SenSize)) ~= 3)
    error(errSensor)
elseif(SenSize(1) <= 0 || SenSize(2) <= 0)
    error(errSensor)
end
if (SenSize(1) > 36 || SenSize(2) > 36)
    error(errFullFrame)
elseif (((SenSize(2) == 36) && (SenSize(1) > 24)) || ((SenSize(1) == 36) && (SenSize(2) > 24)))
    error(errFullFrame)
end

Ovrl = OvrlP/100;
mFocLen = FocLen/1000;
mSenSize = SenSize/1000;

if (mFocLen > FlHeight)
    error(errSwitch)
elseif (mFocLen <= 0)
    error(errFocal)
elseif (FlHeight <= 0)
    error(errHeight)
elseif (OvrlP < 0 || OvrlP > 100)
    error(errOvrl)
end
%%
%vypocet rozmerov fotografie a prislusnej vzdialenosti %photograph size and coresponding distance calculation
PhSize = (FlHeight/mFocLen)*mSenSize;
switch FwdSenPar
    case 1
        OutDist = PhSize(1)*(1-Ovrl);
    case 2
        OutDist = PhSize(2)*(1-Ovrl);
    otherwise
        error(errOrient)
end