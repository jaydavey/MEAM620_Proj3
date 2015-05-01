function [S,G] = getRandomPoints(Ns,Ng,bound,R)

% This function randomly choose points that are within the
% bounds spaced R apart.  bounds may be 2D or 3D

% Inputs: - Ns, Number of starting locations
%         - Ng, Number of goal locations
%         - bound, bounding box that permits acceptable point locations
%                  3D = [xmin xmax ymin ymax zmin zmax]
%                  2D = [xmin xmax ymin ymax]

%         - R, minimum separation of points
%         - timeout, if points haven't been found abort function

% Outputs:    - S, non colliding starting points within bound
%             - G, non colliding goal points within bound

% start the timer for timeout
tic;

S = []; %we don't want to preallocate here
G = []; %we don't want to preallocate here

% figure out if we're doing 2D or 3D
if length(bound)==4
    %We're doing 2D pt generation
    for i = 1:Ns
        % Generate a random point
        pt = [ (bound(2)-bound(1))*rand()+bound(1), (bound(4)-bound(3))*rand()+bound(3)];
        
        % add pt to the list of pts
        if isempty(S);
            % add the first point to the list
            S(1,:) = pt;
        else
            new_point_found = 0;
            while ~new_point_found
                % Check pt doesn't collide with other start pts
                ptlist = repmat(pt,size(S,1),1);
                Rmin = min(pdist2(ptlist,S,'euclidean','Smallest',1));
                if Rmin>(2*R*sqrt(2))
                    new_point_found = 1;
                else
                    % Generate a random point
                    pt = [ (bound(2)-bound(1))*rand()+bound(1), (bound(4)-bound(3))*rand()+bound(3)];
                end
            end
            % add this non-colliding point to list
            S = [S;pt];
        end
    end
    
    for i = 1:Ng
        % Generate a random point
        pt = [ (bound(2)-bound(1))*rand()+bound(1), (bound(4)-bound(3))*rand()+bound(3)];
        
        % add pt to the list of pts
        if isempty(G);
            % add the first point to the list
            G(1,:) = pt;
        else
            new_point_found = 0;
            while ~new_point_found
                % Check pt doesn't collide with other start pts
                ptlist = repmat(pt,size(G,1),1);
                Rmin = min(pdist2(ptlist,G,'euclidean','Smallest',1));
                if Rmin>(2*R*sqrt(2))
                    new_point_found = 1;
                else
                    % Generate a random point
                    pt = [ (bound(2)-bound(1))*rand()+bound(1), (bound(4)-bound(3))*rand()+bound(3)];
                end
            end
            % add this non-colliding point to list
            G = [G;pt];
        end
    end
else
    %We're doing 3D pt generation
    for i = 1:Ns
        % Generate a random point
        pt = [ (bound(2)-bound(1))*rand()+bound(1), (bound(4)-bound(3))*rand()+bound(3), (bound(6)-bound(5))*rand()+bound(5)];
        
        % add pt to the list of pts
        if isempty(S);
            % add the first point to the list
            S(1,:) = pt;
        else
            new_point_found = 0;
            while ~new_point_found
                % Check pt doesn't collide with other start pts
                ptlist = repmat(pt,size(S,1),1);
                Rmin = min(pdist2(ptlist,S,'euclidean','Smallest',1));
                if Rmin>(2*R*sqrt(2))
                    new_point_found = 1;
                else
                    % Generate a random point
                    pt = [ (bound(2)-bound(1))*rand()+bound(1), (bound(4)-bound(3))*rand()+bound(3), (bound(6)-bound(5))*rand()+bound(5)];
                end
            end
            % add this non-colliding point to list
            S = [S;pt];
        end
    end
    
    for i = 1:Ng
        % Generate a random point
        pt = [ (bound(2)-bound(1))*rand()+bound(1), (bound(4)-bound(3))*rand()+bound(3), (bound(6)-bound(5))*rand()+bound(5)];
        
        % add pt to the list of pts
        if isempty(G);
            % add the first point to the list
            G(1,:) = pt;
        else
            new_point_found = 0;
            while ~new_point_found
                % Check pt doesn't collide with other start pts
                ptlist = repmat(pt,size(G,1),1);
                Rmin = min(pdist2(ptlist,G,'euclidean','Smallest',1));
                if Rmin>(2*R*sqrt(2))
                    new_point_found = 1;
                else
                    % Generate a random point
                    pt = [ (bound(2)-bound(1))*rand()+bound(1), (bound(4)-bound(3))*rand()+bound(3), (bound(6)-bound(5))*rand()+bound(5)];
                end
            end
            % add this non-colliding point to list
            G = [G;pt];
        end
    end
end

end