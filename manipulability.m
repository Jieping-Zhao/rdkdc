function mu = manipulability(J, measure)
% inputs: There are two inputs to this function:
%       * J :6*6 matrix
%       * measure: a single string argument that can only be one of 'sigmamin', 'detjac', or
%                'invcond'. Defines which manipulability measure is used.
% output: mu: The corresponding measure of manipulability

% situation 1 : Minimum singular value of J (MLS/CH.4/section 4.4/example 3.14)
    if measure == "sigmamin"
        mu = svds(J, 1,'smallest');
        
% situation 2 : Inverse of the condition number of J (example 3.15)
    elseif measure == "invcond"
        SigmaMax = svds(J, 1, 'largest');
        SigmaMin = svds(J, 1, 'smallest');
        mu = SigmaMin / SigmaMax;
        
% situation 3: Determinant of J (example 3.16)
    elseif measure == "detjac"
        mu = det(J);

% NO argument or wrong argument 
    else
        error("NO argument or Wrong argument");
    end
end
