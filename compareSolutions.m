% Check differences of different solution from VI/PI/Lp

diff_lp_pi = sum((J_opt_lp - J_opt_pi).^2)
diff_lp_vi = sum((J_opt_lp - J_opt_vi).^2)
diff_vi_pi = sum((J_opt_vi - J_opt_pi).^2)

inputDiff_lp_pi = sum(abs((u_opt_ind_lp == u_opt_ind_pi) - 1))
inputDiff_lp_vi = sum(abs((u_opt_ind_lp == u_opt_ind_vi) - 1))
inputDiff_vi_pi = sum(abs((u_opt_ind_vi == u_opt_ind_pi) - 1))

diff = (J_opt_lp - J_opt_vi)
