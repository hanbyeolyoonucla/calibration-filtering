function [T_mat, XYZWPR] = SE32mat(T_SE3)

    p_0 = transl(T_SE3);
    R_0 = t2r(T_SE3);
    eul_0 =  rotm2eul(R_0, 'XYZ')*180/pi;
    T_mat = [R_0 p_0'; zeros(1,3) 1];
    XYZWPR = [p_0 eul_0];
    
end

