function [OutStruct] = shapecreate(InMat,Gtype,id,FileName)
%SHAPECREATE - Create a custom shapefile
%
%   This function creates a custom shapefile specified by the vertex data [InMat], geometry type [Gtype]
%   and assigned id [id] to a file named [FileName].
%
%   [OutStruct] = SHAPECREATE(InMat,Gtype, id, FileName)
%
%   Input data [InMat] must be a matrix of geodetic (polar) coordinates of format [latitude longitude].
%   Geometry type [Gtype] must be one of the followig strings: 
%       'Point', 'MultiPoint', 'Line', 'PolyLine', or 'Polygon'.

%%
%inicializacne parametre %initial parameters
errData = 'Invalid input data (1st parameter). Input must be a two column matrix containing geodetic coordinates. For more info please visit help.';
errType = 'Invalid geometry type (2nd parameter). Only the following strings are allowed: "Point", "MultiPoint", "Line", "PolyLine", or "Polygon". For more info please visit help.';
if (~isnumeric(InMat) || (size(InMat,2) ~= 2 && size(InMat,2) ~= 3))
    error(errData)
end
%%
%vytvorenie struktury a samotneho shapefile-u 
%structure and the actual shapefile creation
OutStruct.Geometry = Gtype;
OutStruct.BoundingBox(:,1) = InMat(:,2);
OutStruct.BoundingBox(:,2) = InMat(:,1);
OutStruct.X = InMat(:,2)';
OutStruct.X = [OutStruct.X, NaN];
OutStruct.Y = InMat(:,1)';
OutStruct.Y = [OutStruct.Y, NaN];
OutStruct.id = id;
try
    shapewrite(OutStruct,FileName);
catch err
    if (strcmp(err.identifier, 'map:geostruct:invalidGeometryString'))
        error(errType)
    else
        error(err.message)
    end
end
end