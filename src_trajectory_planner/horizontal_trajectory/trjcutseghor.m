function [CtrajP,NofWp] = trjcutseghor(InTraj,LenS)
%TRJCUTSEGHOR - Cut trajectory into smaller segments
%
%   This function cuts the input trajectory [InTraj] into segments of length defined by [LenS] and returns
%   the final waypoints in a matrix [CtrajP]. The number of calculated waypoints is returned in [NofWp].
%
%   [CtrajP,NofWp] = TRJCUTSEGHOR(InTraj,LenS)
%
%   The input trajectory [InTraj] must be a matrix of geodetic (polar) coordinates with 3 columns [latitude longitude height]
%   or 2 columns [latitude longitude]. The size of final segments [LenS] must be a positive and non-zero value in meters [m].

%%
%inicializacne parametre %initial parameters
errTraj = 'Invalid input trajectory (1st parameter). Input must be a matrix of polar coordinates with 2 or 3 columns. For more info please visit help.';
errLen = 'Invalid segment length (2nd parameter). Input must be a non-zero positive value. For more info please visit help.';
if (LenS <= 0)
    error(errLen)
end
if (~isnumeric(InTraj))
    error(errTraj)
end
TrajL = zeros(size(InTraj,1),size(InTraj,2));
%%
%orezanie trajektorie na pozadovane segmenty %cuttning trajectory into defined segments
invLenS = 1 / LenS;
switch (size(InTraj,2))
    case 3
        %prevod z polarnej na lokalnu sustavu %conversion from polar to local frame
        [TrajL(:,1),TrajL(:,2),TrajL(:,3)] = geodetic2enu(InTraj(:,1),InTraj(:,2),InTraj(:,3),InTraj(1,1),InTraj(1,2),InTraj(1,3),wgs84Ellipsoid);
        %segmentacia %segmentation
        poz = 1;
        for i = 1:(size(TrajL,1)-1)
            LenH = sqrt((TrajL(i+1,1) - TrajL(i,1))^2 + (TrajL(i+1,2) - TrajL(i,2))^2);
            NofSeg = round(LenH * invLenS);
            if (NofSeg == 0 || NofSeg == 1)
                CtrajL(poz,1) = TrajL(i,1);
                CtrajL(poz,2) = TrajL(i,2);
                CtrajL(poz,3) = TrajL(i,3);
                poz = poz + 1;
            else
                x = linspace(TrajL(i,1), TrajL(i+1,1), NofSeg);
                y = linspace(TrajL(i,2), TrajL(i+1,2), NofSeg);
                z = linspace(TrajL(i,3), TrajL(i+1,3), NofSeg);
                CtrajL(poz:poz+NofSeg-1,1) = x(1:NofSeg);
                CtrajL(poz:poz+NofSeg-1,2) = y(1:NofSeg);
                CtrajL(poz:poz+NofSeg-1,3) = z(1:NofSeg);
                poz = poz + NofSeg-1;
            end
        end
        if (min(CtrajL(end,:) == TrajL(end,:)) ~= 1)
            CtrajL(end+1,:) = TrajL(end,:);
        end
        %finalny prevod z lokalnej na polarnu sustavu %final conversion from local to polar frame
        CtrajP = zeros(size(CtrajL,1),size(CtrajL,2));
        [CtrajP(:,1),CtrajP(:,2),CtrajP(:,3)] = enu2geodetic(CtrajL(:,1),CtrajL(:,2),CtrajL(:,3),InTraj(1,1),InTraj(1,2),InTraj(1,3),wgs84Ellipsoid);
    case 2
        %prevod z polarnej na lokalnu sustavu %conversion from polar to local frame
        [TrajL(:,1),TrajL(:,2)] = geodetic2enu(InTraj(:,1),InTraj(:,2),0,InTraj(1,1),InTraj(1,2),0,wgs84Ellipsoid);
        %segmentacia %segmentation
        poz = 1;
        for i = 1:(size(TrajL,1)-1)
            LenH = sqrt((TrajL(i+1,1) - TrajL(i,1))^2 + (TrajL(i+1,2) - TrajL(i,2))^2);
            NofSeg = round(LenH * invLenS);
            if (NofSeg == 0 || NofSeg == 1)
                CtrajL(poz,1) = TrajL(i,1);
                CtrajL(poz,2) = TrajL(i,2);
                poz = poz + 1;
            else
                x = linspace(TrajL(i,1), TrajL(i+1,1), NofSeg);
                y = linspace(TrajL(i,2), TrajL(i+1,2), NofSeg);
                CtrajL(poz:poz+NofSeg-1,1) = x(1:NofSeg);
                CtrajL(poz:poz+NofSeg-1,2) = y(1:NofSeg);
                poz = poz + NofSeg-1;
            end
        end
        if (min(CtrajL(end,:) == TrajL(end,:)) ~= 1)
            CtrajL(end+1,:) = TrajL(end,:);
        end
        %finalny prevod z lokalnej na polarnu sustavu %final conversion from local to polar frame
        CtrajP = zeros(size(CtrajL,1),size(CtrajL,2));
        [CtrajP(:,1),CtrajP(:,2)] = enu2geodetic(CtrajL(:,1),CtrajL(:,2),0,InTraj(1,1),InTraj(1,2),0,wgs84Ellipsoid);
    otherwise
        error(errTraj)
end
%pocet finalnych bodov %number of final waypoints
NofWp = size(CtrajP,1);
end