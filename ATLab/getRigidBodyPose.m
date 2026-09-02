function pose2d = getRigidBodyPose(streamingID)
% Returns [x y z yaw_deg] for the given Motive rigid body ID.
% Ground-plane: XY (Z up). X is flipped. Yaw in [0,360).

    persistent client P
    if isempty(client)
        client   = natnet;
        serverIP = '192.168.1.170';   % Motive PC
        clientIP = '192.168.1.222';   % This PC
        client.ConnectToNatNet(clientIP, serverIP, 'Multicast');

        % Axis permutation: [Xr;Yr;Zr] = P * [Xm;Ym;Zm] with Xr=Xm, Yr=Zm, Zr=Ym
        P = [1 0 0; 0 0 1; 0 1 0];
    end

    % ---- Get latest frame ----
    frame = client.getFrame();
    if isempty(frame) || frame.nRigidBodies < 1
        pose2d = [0 0 0 0]; return;
    end

    % ---- Find the rigid body by ID ----
    idx = findMatchingDataID(frame, streamingID);
    if idx < 1
        pose2d = [0 0 0 0]; return;
    end

    rb = frame.RigidBodies(idx);

    % ---- Position: Motive (X,Z,Y) -> Robot (X,Y,Z), then flip X ----
    xr = -double(rb.x);   % flip X
    yr =  double(rb.z);
    zr =  double(rb.y);

    % ---- Orientation: quaternion -> R (Motive) -> R (Robot) ----
    qx = double(rb.qx); qy = double(rb.qy); qz = double(rb.qz); qw = double(rb.qw);
    nq = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    if nq > 0, qw=qw/nq; qx=qx/nq; qy=qy/nq; qz=qz/nq; end

    Rm = quat_to_R(qw,qx,qy,qz);
    Rr = P * Rm * P.';     % express in XY, Z-up frame

    % ---- Heading from forward axis (body X) with same X flip ----
    fwd = Rr(:,1);                  % body X in world coords
    fx  = -fwd(1);                  % flip X
    fy  =  fwd(2);
    yaw_deg = mod(atan2(fy, fx) * (180/pi), 360);

    pose2d = [xr, yr, zr, yaw_deg];
end

% ===================== Helpers (same file) =====================

function idx = findMatchingDataID(data, rigidID)
% Return index of RigidBodies(i) whose ID equals rigidID; -1 if not found.
    idx = -1;
    for i = 1:data.nRigidBodies
        if rigidID == data.RigidBodies(i).ID
            idx = i; return;
        end
    end
end

function R = quat_to_R(w,x,y,z)
% Rotation matrix from unit quaternion (w,x,y,z).
    R = [1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w);
         2*(x*y + z*w),   1-2*(x*x+z*z),   2*(y*z - x*w);
         2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x*x+y*y)];
end
