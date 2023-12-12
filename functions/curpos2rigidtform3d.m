function pose = curpos2rigidtform3d(curpos)
    %CURPOS2RIGIDTFORM3D convert the read out current pose (tx,ty,tz,rx,ry,rz)
    % of the cobotta into a rigid3dtform object 
    % 
    % rot in euler angles: Rz*Ry*Rx
    
    pose = rigidtform3d(curpos(4:6),curpos(1:3));
end

