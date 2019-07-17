function [FinCoorP] = trjgenhor(InPolygonP,LineDist,RefPoint,PolyFill,varargin)
%TRJGENHOR - Generate horizontal trajectory
%
%   This function returns a matrix of geodetic (polar) coordinates [FinCoorP] of
%   points corresponding to the given distance between trajectory lines [LineDist] and the chosen
%   reference point [RefPoint] within a polygon specified by a matrix [InPolygonP] of boundary coordinates.
%   The optional [NumofLines LOffset] parameter explicitly specifies the the number of generated lines and
%   an offset by which the trajectory should be shifted from origin.
%
%   [FinCoorP] = TRJGENHOR(InPolygonP,LineDist,RefPoint,PolyFill)
%   [FinCoorP] = TRJGENHOR(InPolygonP,LineDist,RefPoint,PolyFill,NumofLines)
%   [FinCoorP] = TRJGENHOR(InPolygonP,LineDist,RefPoint,PolyFill,[NumofLines LOffset])
%
%   The input matrix must be a 4x2 matrix where each row represents a geodetic (polar) coorinate [latitude longitude].
%   All four verteces of the polygon must have different coordinates in space. Interior angles of the polygon must not
%   be larger than 180°. To fill the entire area of the polygon the [PolyFill] parameter must be set to true.
%   The optional parameter can either be a single value [NumofLines] or a two-value vector [NumofLines LOffset].
%   [NumofLines] must be a positive non-zero integer not greater than the maximum number of lines allowed by the size
%   of the input polygon. [LOffset] must be a positive number in meters [m] which is limited by the maximum distance
%   allowed by the size of the input polygon and the size of the trajectory.
%   

%%
%inicializacne parametre
%initial parameters
errPoly = 'Invalid polygon (1st parameter). Input must be a 4x2 matrix of polar coordinates. For more info please visit help.';
errDist = 'Invalid distance between trajectory lines (2nd parameter). Input must be a non-zero positive value. For more info please visit help.';
errPoint = 'Invalid reference point (3rd parameter). Input must be an integer from 1 to 4. For more info please visit help.';
errAngle = 'The polygon has an unallowed shape. Please make sure all angles inside the polygon are < 180°. For more info please visit help.';
errVertex = 'The polygon has an unallowed shape. Please make sure all verteces of the polygon represent different coordinates. For more info please visit help.';
errFill = 'Invalid fourth parameter. Input must be a bool value. For more info please visit help.';
errOptPar = 'Invalid optional parameter. Input must be a vector with a non-zero positive integer and a positive value or a single non-zero positive integer. For more info please visit help.';
errOptSize = 'Invalid optional parameter. Please reduce the number of lines or the offset. For more info please visit help.';
RefP = InPolygonP;
if (size(RefP,1) ~= 4 || size(RefP,2) ~= 2 || ~isnumeric(RefP))
    error(errPoly)
end
if (size(unique(RefP,'rows'),1) ~= 4)
    error(errVertex)
end
if (LineDist <= 0)
    error(errDist)
end
if (RefPoint ~= 1 && RefPoint ~= 2 && RefPoint ~= 3 && RefPoint ~= 4)
    error(errPoint)
end
if (PolyFill ~= true && PolyFill ~= false)
    error(errFill)
end
if (size(varargin,2) > 1)
    error('Too many input arguments.')
end
if (nargin == 5)
    if ((~isnumeric(varargin{1})) || (sum(size(varargin{1})) > 3) || (mod(varargin{1}(1),1) ~= 0) || (varargin{1}(1) <= 0) || (varargin{1}(2) < 0))
        error(errOptPar)
    end
    EXNumofLines = varargin{1}(1);
    if (sum(size(varargin{1})) == 2)
        LOffset = 0;
    else
        LOffset = varargin{1}(2);
    end
end
RefL = zeros(size(RefP,1),size(RefP,2));
%%
%detekcia reflexneho uhlu v polygone %detection of a reflex angle inside the polygon
for i = 1:size(RefP,1)
    [RefL(:,1),RefL(:,2)] = geodetic2enu(RefP(:,1),RefP(:,2),0,RefP(i,1),RefP(i,2),0,wgs84Ellipsoid);
    for j = 1:i-1
       RefL = [RefL; RefL(1,1) RefL(1,2)];
       RefL(1,:) = [];
    end
    %overenie vekosti uhlu %angle size check
    if ((RefL(3,1) < 0 && RefL(3,2) < 0) || (RefL(3,1) < 0 && RefL(3,2) > 0))
        if (RefL(2,1) > RefL(4,1))
            if (abs(atan(RefL(4,2)/RefL(4,1)) + atan(RefL(2,1)/RefL(2,2))) > pi/2)
                error(errAngle)
            end
        else
            if (abs(atan(RefL(2,2)/RefL(2,1)) + atan(RefL(4,1)/RefL(4,2))) > pi/2)
                error(errAngle)
            end
        end  
    elseif ((RefL(3,1) > 0 && RefL(3,2) > 0) || (RefL(3,1) > 0 && RefL(3,2) < 0))
        if (RefL(2,1) > RefL(4,1))
            if (abs(atan(RefL(4,1)/RefL(4,2)) + atan(RefL(2,2)/RefL(2,1))) > pi/2)
                error(errAngle)
            end
        else
            if (abs(atan(RefL(2,1)/RefL(2,2)) + atan(RefL(4,2)/RefL(4,1))) > pi/2)
                error(errAngle)
            end
        end  
    end     
end
RefL = zeros(size(RefP,1),size(RefP,2));
%%
%prevod z polarnej na lokalnu sustavu s referencnym bodom na zvolenom pociatku
%conversion from polar to local frame with a specific origin
[RefL(:,1),RefL(:,2)] = geodetic2enu(RefP(:,1),RefP(:,2),0,RefP(RefPoint,1),RefP(RefPoint,2),0,wgs84Ellipsoid);
%korekcia suradnic vzhladom na pociatok %frame correction on origin
for i = 1:RefPoint-1
   RefL = [RefL; RefL(1,1) RefL(1,2)];
   RefL(1,:) = [];
end
%%
%rotacia polygonu %polygon rotation
angleR = atan(RefL(2,2)/RefL(2,1));
R = [cos(-angleR) -sin(-angleR) 0; sin(-angleR) cos(-angleR) 0; 0 0 1];
RefLtmp = RefL';
RefLtmp(3,:) = 0;
newRefL = (R*RefLtmp)';
%korekcia orientacie polygonu %polygon orientation correction
if (newRefL(3,2) < 0 && newRefL(4,2) < 0)
    angleR = angleR + pi;
    R = [cos(-angleR) -sin(-angleR) 0; sin(-angleR) cos(-angleR) 0; 0 0 1];
    newRefL = (R*RefLtmp)';
end
%%
%korekcia ku postrannej ciare %side line latch correction
BB = sqrt((newRefL(3,2)-newRefL(2,2))^2 + (newRefL(2,1)-newRefL(3,1))^2);
CC = sqrt((newRefL(4,2)-newRefL(3,2))^2 + (newRefL(3,1)-newRefL(4,1))^2);
DD = sqrt(newRefL(4,1)^2 + newRefL(4,2)^2);

corrDiffB = newRefL(3,1) - newRefL(2,1);
corrAngB = asin(abs(corrDiffB)/BB);
corrsignB = sign(corrDiffB);

corrDiffC = newRefL(4,1) - newRefL(3,1);
corrAngC = asin(abs(corrDiffC)/CC);
corrsignC = sign(corrDiffC);

corrDiffD = newRefL(4,1);
corrAngD = asin(abs(corrDiffD)/DD);
corrsignD = sign(corrDiffD);
%%
%pocet trasovych ciar %number of trajectory lines
if (PolyFill)
    if (cos(corrAngD)*DD < cos(corrAngB)*BB)
        NumofLines = ceil(cos(corrAngB)*BB/LineDist);
    else
        NumofLines = ceil(cos(corrAngD)*DD/LineDist);
    end
else
    if (cos(corrAngD)*DD > cos(corrAngB)*BB)
        NumofLines = ceil(cos(corrAngB)*BB/LineDist);
    else
        NumofLines = ceil(cos(corrAngD)*DD/LineDist);
    end
end
%%
%explicitne zadany pocet ciar a offset trajektorie
%explicit number of lines and the trajectory offset
if (nargin == 5)
    MAXDist = (NumofLines-1)*LineDist;
    NewDist = (EXNumofLines-1)*LineDist + LOffset;
    if ((EXNumofLines > NumofLines) || (NewDist > MAXDist))
        error(errOptSize)
    end
    NumofLines = EXNumofLines;
    
    if (((newRefL(2,2) + 1*LOffset) > newRefL(3,2)) && PolyFill)
        newRefL(2,1:2) = [(newRefL(2,1) + ((newRefL(2,2) + 1*LOffset) - newRefL(3,2))*tan(corrAngC)*corrsignC + (newRefL(3,1) - newRefL(2,1))) (newRefL(2,2) + 1*LOffset)];
    else
        newRefL(2,1:2) = [(newRefL(2,1) + 1*LOffset*tan(corrAngB)*corrsignB) (newRefL(2,2) + 1*LOffset)];
    end
    newRefL(1,1:2) = [(newRefL(1,1) + 1*LOffset*tan(corrAngD)*corrsignD) (newRefL(1,2) + 1*LOffset)];
    
    BB = sqrt((newRefL(3,2)-newRefL(2,2))^2 + (newRefL(2,1)-newRefL(3,1))^2);
    CC = sqrt((newRefL(4,2)-newRefL(3,2))^2 + (newRefL(3,1)-newRefL(4,1))^2);
    DD = sqrt(newRefL(4,1)^2 + newRefL(4,2)^2);

    corrDiffB = newRefL(3,1) - newRefL(2,1);
    corrAngB = asin(abs(corrDiffB)/BB);
    corrsignB = sign(corrDiffB);

    corrDiffC = newRefL(4,1) - newRefL(3,1);
    corrAngC = asin(abs(corrDiffC)/CC);
    corrsignC = sign(corrDiffC);

    corrDiffD = newRefL(4,1);
    corrAngD = asin(abs(corrDiffD)/DD);
    corrsignD = sign(corrDiffD);
end
%%
%generovanie finalnych bodov %generating final points
if (cos(corrAngD)*DD > cos(corrAngB)*BB)
    if (NumofLines == 1)
        FinL = [newRefL(1,1) newRefL(1,2)];
        FinL = [FinL; newRefL(2,1) newRefL(2,2)];
    elseif (NumofLines == 2)
        FinL = [newRefL(1,1) newRefL(1,2)];
        FinL = [FinL; newRefL(2,1) newRefL(2,2)];
        if (((newRefL(2,2) + 1*LineDist) > newRefL(3,2)) && PolyFill)
            FinL = [FinL; (newRefL(2,1) + ((newRefL(2,2) + 1*LineDist) - newRefL(3,2))*tan(corrAngC)*corrsignC + (newRefL(3,1) - newRefL(2,1))) (newRefL(2,2) + 1*LineDist)];
        else
            FinL = [FinL; (newRefL(2,1) + 1*LineDist*tan(corrAngB)*corrsignB) (newRefL(2,2) + 1*LineDist)];
        end
        FinL = [FinL; (newRefL(1,1) + 1*LineDist*tan(corrAngD)*corrsignD) (newRefL(1,2) + 1*LineDist)];
    elseif (mod(NumofLines,2) == 1)
        FinL = [newRefL(1,1) newRefL(1,2)];
        FinL = [FinL; newRefL(2,1) newRefL(2,2)];
        %pocet opakovani sekvencie %sequence repetition calculation
        tmp = 0;
        for i = 3:2:NumofLines
            controlparam = NumofLines-2-tmp;
            tmp = tmp + 1;
        end
        %cyklus sekvencie %sequence cycle
        for i = 0:controlparam-1
            if (((newRefL(2,2) + (1+i*2)*LineDist) > newRefL(3,2)) && PolyFill)
                FinL = [FinL; (newRefL(2,1) + ((newRefL(2,2) + (1+i*2)*LineDist)-newRefL(3,2))*tan(corrAngC)*corrsignC + (newRefL(3,1) - newRefL(2,1))) (newRefL(2,2) + (1+i*2)*LineDist)];
            else
                FinL = [FinL; (newRefL(2,1) + (1+i*2)*LineDist*tan(corrAngB)*corrsignB) (newRefL(2,2) + (1+i*2)*LineDist)];
            end
            FinL = [FinL; (newRefL(1,1) + (1+i*2)*LineDist*tan(corrAngD)*corrsignD) (newRefL(1,2) + (1+i*2)*LineDist)];
            FinL = [FinL; (newRefL(1,1) + (2+i*2)*LineDist*tan(corrAngD)*corrsignD) (newRefL(1,2) + (2+i*2)*LineDist)];
            if (((newRefL(2,2) + (2+i*2)*LineDist) > newRefL(3,2)) && PolyFill)
                FinL = [FinL; (newRefL(2,1) + ((newRefL(2,2) + (2+i*2)*LineDist)-newRefL(3,2))*tan(corrAngC)*corrsignC + (newRefL(3,1) - newRefL(2,1))) (newRefL(2,2) + (2+i*2)*LineDist)];%%
            else
                FinL = [FinL; (newRefL(2,1) + (2+i*2)*LineDist*tan(corrAngB)*corrsignB) (newRefL(2,2) + (2+i*2)*LineDist)];
            end
        end
    elseif (mod(NumofLines,2) == 0)
        FinL = [newRefL(1,1) newRefL(1,2)];
        FinL = [FinL; newRefL(2,1) newRefL(2,2)];
        %pocet opakovani sekvencie %sequence repetition calculation
        tmp = 0;
        for i = 4:2:NumofLines
            controlparam = NumofLines-3-tmp;
            tmp = tmp + 1;
        end
        %cyklus sekvencie %sequence cycle
        for i = 0:controlparam-1
            if (((newRefL(2,2) + (1+i*2)*LineDist) > newRefL(3,2)) && PolyFill)
                FinL = [FinL; (newRefL(2,1) + ((newRefL(2,2) + (1+i*2)*LineDist) - newRefL(3,2))*tan(corrAngC)*corrsignC + (newRefL(3,1) - newRefL(2,1))) (newRefL(2,2) + (1+i*2)*LineDist)];
            else
                FinL = [FinL; (newRefL(2,1) + (1+i*2)*LineDist*tan(corrAngB)*corrsignB) (newRefL(2,2) + (1+i*2)*LineDist)];
            end
            FinL = [FinL; (newRefL(1,1) + (1+i*2)*LineDist*tan(corrAngD)*corrsignD) (newRefL(1,2) + (1+i*2)*LineDist)];
            FinL = [FinL; (newRefL(1,1) + (2+i*2)*LineDist*tan(corrAngD)*corrsignD) (newRefL(1,2) + (2+i*2)*LineDist)];
            if (((newRefL(2,2) + (2+i*2)*LineDist) > newRefL(3,2)) && PolyFill)
                FinL = [FinL; (newRefL(2,1) + ((newRefL(2,2) + (2+i*2)*LineDist) - newRefL(3,2))*tan(corrAngC)*corrsignC + (newRefL(3,1) - newRefL(2,1))) (newRefL(2,2) + (2+i*2)*LineDist)];
            else
                FinL = [FinL; (newRefL(2,1) + (2+i*2)*LineDist*tan(corrAngB)*corrsignB) (newRefL(2,2) + (2+i*2)*LineDist)];
            end
        end
        if (((newRefL(2,2) + (NumofLines-1)*LineDist) > newRefL(3,2)) && PolyFill)
            FinL = [FinL; (newRefL(2,1) + ((newRefL(2,2) + (NumofLines-1)*LineDist) - newRefL(3,2))*tan(corrAngC)*corrsignC + (newRefL(3,1) - newRefL(2,1))) (newRefL(2,2) + (NumofLines-1)*LineDist)];
        else
            FinL = [FinL; (newRefL(2,1) + (NumofLines-1)*LineDist*tan(corrAngB)*corrsignB) (newRefL(2,2) + (NumofLines-1)*LineDist)];
        end
        FinL = [FinL; (newRefL(1,1) + (NumofLines-1)*LineDist*tan(corrAngD)*corrsignD) (newRefL(1,2) + (NumofLines-1)*LineDist)];
    end
else
    if (NumofLines == 1)
        FinL = [newRefL(1,1) newRefL(1,2)];
        FinL = [FinL; newRefL(2,1) newRefL(2,2)];
    elseif (NumofLines == 2)
        FinL = [newRefL(1,1) newRefL(1,2)];
        FinL = [FinL; newRefL(2,1) newRefL(2,2)];
        FinL = [FinL; (newRefL(2,1) + 1*LineDist*tan(corrAngB)*corrsignB) (newRefL(2,2) + 1*LineDist)];
        if (((newRefL(2,2) + (NumofLines-1)*LineDist) > newRefL(4,2)) && PolyFill)
            FinL = [FinL; (newRefL(1,1) + ((newRefL(1,2) + 1*LineDist) - newRefL(4,2))*tan(corrAngC)*-corrsignC) + (newRefL(4,1) - newRefL(1,1)) (newRefL(1,2) + 1*LineDist)];
        else
            FinL = [FinL; (newRefL(1,1) + 1*LineDist*tan(corrAngD)*corrsignD) (newRefL(1,2) + 1*LineDist)];
        end
    elseif (mod(NumofLines,2) == 1)
        FinL = [newRefL(1,1) newRefL(1,2)];
        FinL = [FinL; newRefL(2,1) newRefL(2,2)];
        %pocet opakovani sekvencie %sequence repetition calculation
        tmp = 0;
        for i = 3:2:NumofLines
            controlparam = NumofLines-2-tmp;
            tmp = tmp + 1;
        end
        %cyklus sekvencie %sequence cycle
        for i = 0:controlparam-1
            FinL = [FinL; (newRefL(2,1) + (1+i*2)*LineDist*tan(corrAngB)*corrsignB) (newRefL(2,2) + (1+i*2)*LineDist)];
            if (((newRefL(1,2) + (1+i*2)*LineDist) > newRefL(4,2)) && PolyFill)
                FinL = [FinL; (newRefL(1,1) + ((newRefL(1,2) + (1+i*2)*LineDist) - newRefL(4,2))*tan(corrAngC)*-corrsignC) + (newRefL(4,1) - newRefL(1,1)) (newRefL(1,2) + (1+i*2)*LineDist)];
            else
                FinL = [FinL; (newRefL(1,1) + (1+i*2)*LineDist*tan(corrAngD)*corrsignD) (newRefL(1,2) + (1+i*2)*LineDist)];
            end
            if (((newRefL(1,2) + (2+i*2)*LineDist) > newRefL(4,2)) && PolyFill)
                FinL = [FinL; (newRefL(1,1) + ((newRefL(1,2) + (2+i*2)*LineDist) - newRefL(4,2))*tan(corrAngC)*-corrsignC) + (newRefL(4,1) - newRefL(1,1)) (newRefL(1,2) + (2+i*2)*LineDist)];
            else
                FinL = [FinL; (newRefL(1,1) + (2+i*2)*LineDist*tan(corrAngD)*corrsignD) (newRefL(1,2) + (2+i*2)*LineDist)];
            end
            FinL = [FinL; (newRefL(2,1) + (2+i*2)*LineDist*tan(corrAngB)*corrsignB) (newRefL(2,2) + (2+i*2)*LineDist)];
        end
    elseif (mod(NumofLines,2) == 0)
        FinL = [newRefL(1,1) newRefL(1,2)];
        FinL = [FinL; newRefL(2,1) newRefL(2,2)];
        %pocet opakovani sekvencie %sequence repetition calculation
        tmp = 0;
        for i = 4:2:NumofLines
            controlparam = NumofLines-3-tmp;
            tmp = tmp + 1;
        end
        %cyklus sekvencie %sequence cycle
        for i = 0:controlparam-1
            FinL = [FinL; (newRefL(2,1) + (1+i*2)*LineDist*tan(corrAngB)*corrsignB) (newRefL(2,2) + (1+i*2)*LineDist)];
            if (((newRefL(1,2) + (1+i*2)*LineDist) > newRefL(4,2)) && PolyFill)
                FinL = [FinL; (newRefL(1,1) + ((newRefL(1,2) + (1+i*2)*LineDist) - newRefL(4,2))*tan(corrAngC)*-corrsignC) + (newRefL(4,1) - newRefL(1,1)) (newRefL(1,2) + (1+i*2)*LineDist)];
            else
                FinL = [FinL; (newRefL(1,1) + (1+i*2)*LineDist*tan(corrAngD)*corrsignD) (newRefL(1,2) + (1+i*2)*LineDist)];
            end
            if (((newRefL(1,2) + (2+i*2)*LineDist) > newRefL(4,2)) && PolyFill)
                FinL = [FinL; (newRefL(1,1) + ((newRefL(1,2) + (2+i*2)*LineDist) - newRefL(4,2))*tan(corrAngC)*-corrsignC) + (newRefL(4,1) - newRefL(1,1)) (newRefL(1,2) + (2+i*2)*LineDist)];
            else
                FinL = [FinL; (newRefL(1,1) + (2+i*2)*LineDist*tan(corrAngD)*corrsignD) (newRefL(1,2) + (2+i*2)*LineDist)];
            end
            FinL = [FinL; (newRefL(2,1) + (2+i*2)*LineDist*tan(corrAngB)*corrsignB) (newRefL(2,2) + (2+i*2)*LineDist)];
        end
        FinL = [FinL; (newRefL(2,1) + (NumofLines-1)*LineDist*tan(corrAngB)*corrsignB) (newRefL(2,2) + (NumofLines-1)*LineDist)];
        if (((newRefL(1,2) + (NumofLines-1)*LineDist) > newRefL(4,2)) && PolyFill)
            FinL = [FinL; (newRefL(1,1) + ((newRefL(1,2) + (NumofLines-1)*LineDist) - newRefL(4,2))*tan(corrAngC)*-corrsignC) + (newRefL(4,1) - newRefL(1,1)) (newRefL(1,2) + (NumofLines-1)*LineDist)];
        else
            FinL = [FinL; (newRefL(1,1) + (NumofLines-1)*LineDist*tan(corrAngD)*corrsignD) (newRefL(1,2) + (NumofLines-1)*LineDist)];
        end
   end
end
%%
%spätna rotacia finalnych hodnot do spravnej polohy
%reverse rotation of final points to the correct position
R = [cos(angleR) -sin(angleR) 0; sin(angleR) cos(angleR) 0; 0 0 1];
FinLtmp = FinL';
FinLtmp(3,:) = 0;
newFinL = (R*FinLtmp)';
%%
%konecny prevod finalnych hodnot z lokalnej do polarnej sustavy
%final points conversion from local to polar frame
FinCoorP = zeros(size(newFinL,1),2);
[FinCoorP(:,1),FinCoorP(:,2)] = enu2geodetic(newFinL(:,1),newFinL(:,2),0,RefP(RefPoint,1),RefP(RefPoint,2),0,wgs84Ellipsoid);
end