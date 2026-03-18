function render_aircraft(compData, CG, tilt_angle)
    % Renders the initialized VTOL aircraft based on compData

    N = size(compData, 1);

    % Setup the 3D figure environment
    figure('Name', 'VTOL Aircraft Render', 'Color',[0.1 0.1 0.1],'Position',[50 50 1400 900]); 
    hold on; grid on; axis equal;
    set(gca, 'ZDir', 'reverse', 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w'); 
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title(sprintf('Aircraft Render (Tilt: %d deg)', tilt_angle), 'Color', 'w', 'FontSize', 14);

    % Loop through every component in compData and draw it
    for i = 1:N
        name = compData{i, 1}; 
        type = compData{i, 2};
        dim  = compData{i, 4}; 
        pos  = compData{i, 5}; 
        eul  = compData{i, 6} * (pi/180);
        
        % Assign colors based on component name/type
        if strcmp(type, 'fuselage')
            color = [0.0 1.0 1.0]; 
        elseif contains(name, 'Ballast')
            color = [1.0 1.0 0.0]; 
        elseif contains(name, 'Wing')
            color = [1.0 0.0 1.0]; 
        elseif contains(name, 'Tail')
            color = [1.0 0.5 0.0]; 
        elseif contains(name, 'Boom') || contains(name, 'Arm')
            color = [0.0 1.0 0.0]; 
        elseif contains(name, 'F-Rotor')
            color = [1.0 0.0 0.0]; 
        elseif contains(name, 'R-Rotor')
            color = [0.0 0.5 1.0]; 
        else
            color = [0.5 0.5 0.5]; % Fallback grey
        end
        
        % Generate local vertices and faces
        if strcmp(type, 'fuselage')
            [V, F] = genFacetedFuselage(dim(1), dim(2), dim(3));
        elseif strcmp(type, 'box')
            [V, F] = genCleanBox(dim(1), dim(2), dim(3));
        elseif strcmp(type, 'crossprop')
            [V, F] = genCrossProp(dim(1), dim(2), dim(3)); 
        end
        
        % Rotation Matrices
        phi = eul(1); theta = eul(2); psi = eul(3);
        Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
        Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
        Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
        
        % Apply rotation and translation
        V_trans = ((Rz * Ry * Rx) * V')' + pos; 
        
        % Render the patch
        patch('Vertices', V_trans, 'Faces', F, ...
            'FaceColor', color, 'FaceAlpha', 0.85, 'EdgeColor', 'k', 'LineWidth', 1.0);
    end

    % Plot CG marker
    plot3(CG(1), CG(2), CG(3), 'wp', 'MarkerFaceColor','w', 'MarkerSize',15, 'LineWidth', 2);
    text(CG(1), CG(2), CG(3)-0.6, '  CG', 'FontWeight','bold', 'FontSize', 12, 'Color', 'w');

    view(130, 25); camlight right; lighting flat; hold off;
end

% =========================================================
% GEOMETRY HELPER FUNCTIONS (Local to this script)
% =========================================================
function [V, F] = genCleanBox(L, W, H)
    x = L/2; y = W/2; z = H/2;
    V = [-x -y -z;  x -y -z;  x  y -z; -x  y -z; 
         -x -y  z;  x -y  z;  x  y  z; -x  y  z];
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
end

function [V, F] = genCrossProp(D, W, T)
    [V1, F1] = genCleanBox(D, W, T);
    [V2, F2] = genCleanBox(W, D, T);
    V = [V1; V2];
    F = [F1; F2 + 8]; 
end

function [V, F] = genFacetedFuselage(L, W, H)
    x_norm = [0.5, 0.35, 0.05, -0.3, -0.5];
    w_norm = [0.3, 0.8, 1.0, 0.6, 0.2];
    z_top_norm = [-0.2, -0.7, -1.0, -0.6, -0.3];
    z_bot_norm = [ 0.4,  0.8,  1.0,  0.5, -0.1];
    n_sec = length(x_norm); V = [];
    for i = 1:n_sec
        x = x_norm(i) * L; y = w_norm(i) * W / 2;
        zT = z_top_norm(i) * H / 2; zB = z_bot_norm(i) * H / 2;
        V = [V; x, y, zT; x, -y, zT; x, -y, zB; x, y, zB];
    end
    F = [];
    for i = 1:(n_sec-1)
        idx = (i-1)*4;
        F = [F; idx+1, idx+2, idx+6, idx+5]; F = [F; idx+2, idx+3, idx+7, idx+6]; 
        F = [F; idx+3, idx+4, idx+8, idx+7]; F = [F; idx+4, idx+1, idx+5, idx+8]; 
    end
    F = [F; 1, 2, 3, 4]; idx = (n_sec-1)*4; F = [F; idx+4, idx+3, idx+2, idx+1]; 
end