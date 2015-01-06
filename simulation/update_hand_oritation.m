q_delta = q_goal*q_hand(j).inverse;
[ang_delta RA_hand] = q_delta.AngleAxis;

if ang_delta > omega_hand*t_int
    q_hand(j+1) = quaternion.angleaxis(omega_hand*t_int, RA_hand) * q_hand(j);
else
    q_hand(j+1) = q_goal;
end