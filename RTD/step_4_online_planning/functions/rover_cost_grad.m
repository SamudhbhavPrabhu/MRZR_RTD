function cost_grad = rover_cost_grad(k_1,psi_end,k_3,w0_max,w0_min,v_max,v_min,x_des,y_des,cx,cy)
%ROVER_COST_GRAD
%    COST_GRAD = ROVER_COST_GRAD(K_1,PSI_END,K_3,W0_MAX,W0_MIN,V_MAX,V_MIN,X_DES,Y_DES,CX,CY)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    11-Mar-2020 22:33:01

t2 = k_1+1.0;
t3 = k_3+1.0;
t4 = v_min.*2.0;
t5 = -x_des;
t6 = psi_end./4.0;
t7 = v_min./2.0;
t8 = v_max./2.0;
t9 = w0_min./2.0;
t10 = w0_min./4.0;
t11 = w0_max./2.0;
t12 = w0_max./4.0;
t17 = psi_end.*7.65e-2;
t18 = psi_end.*3.825e-2;
t19 = w0_min.*7.65e-2;
t20 = w0_min.*3.825e-2;
t21 = w0_max.*3.825e-2;
t13 = -t8;
t14 = -t9;
t15 = -t11;
t16 = -t12;
t22 = -t17;
t23 = -t19;
t24 = -t21;
t25 = t7+t13;
t26 = t9+t15;
t27 = t10+t16;
t37 = t20+t24;
t28 = t3.*t25;
t29 = t2.*t26;
t30 = t28.*2.0;
t31 = -t28;
t33 = -t29;
t34 = t29./2.0;
t38 = (t29-w0_min).^2;
t39 = -(t29-w0_min).^3;
t40 = t29.*7.65e-2;
t46 = (t28-v_min).*(t29-w0_min).*2.0;
t49 = (t28-v_min).*(t29-w0_min).^3.*(2.0./3.0);
t32 = -t30;
t35 = t31+v_min;
t36 = t33+w0_min;
t41 = -t40;
t42 = t6+t14+t34;
t47 = -t46;
t48 = t18+t23+t40;
t50 = t38.*(t28-v_min).*(-4.0./3.0);
t51 = t38.*(t28-v_min).*(4.0./3.0);
t43 = t42.^2;
t44 = t42.^3;
t45 = t19+t41;
t52 = t42.*(t28-v_min).*(-8.0./3.0);
t53 = t42.*(t28-v_min).*(8.0./3.0);
t61 = t38.*t48.*2.0;
t62 = t48.*(t29-w0_min).*(-8.0./3.0);
t64 = t48.*(t29-w0_min).*(8.0./3.0);
t66 = t42.*(t28-v_min).*(t29-w0_min).*4.0;
t67 = t48.*(t29-w0_min).^3.*(-1.6e+1./1.5e+1);
t72 = t38.*t42.*(t28-v_min).*(-1.6e+1./5.0);
t73 = t42.*t48.*4.0;
t80 = t42.*t48.*(t29-w0_min).*(-3.2e+1./5.0);
t81 = t38.*t42.*t48.*(1.6e+1./3.0);
t54 = t43.*(t28-v_min).*(-1.6e+1./5.0);
t55 = t45.*(t29-w0_min).*-2.0;
t56 = t43.*(t28-v_min).*(1.6e+1./5.0);
t57 = t45.*(t29-w0_min).*2.0;
t58 = t44.*(t28-v_min).*(-6.4e+1./2.1e+1);
t59 = t45.*(t29-w0_min).^3.*(-2.0./3.0);
t60 = t38.*t45.*(4.0./3.0);
t63 = t42.*t45.*(8.0./3.0);
t68 = t43.*t45.*(1.6e+1./5.0);
t69 = -t66;
t70 = t44.*t45.*(6.4e+1./2.1e+1);
t71 = t43.*(t28-v_min).*(t29-w0_min).*(1.6e+1./3.0);
t74 = -t73;
t75 = t43.*t48.*(1.6e+1./3.0);
t76 = t44.*t48.*(1.6e+1./3.0);
t77 = t42.*t45.*(t29-w0_min).*-4.0;
t78 = t43.*t45.*(t29-w0_min).*(-1.6e+1./3.0);
t79 = t38.*t42.*t45.*(1.6e+1./5.0);
t82 = t43.*t48.*(t29-w0_min).*(-6.4e+1./7.0);
t65 = -t63;
t83 = t22+t47+t49+t53+t58+t60+t61+t68+t71+t72+t75+t77+t80+y_des;
t84 = t4+t5+t32+t51+t56+t57+t59+t64+t65+t67+t69+t70+t74+t76+t78+t79+t81+t82;
cost_grad = [cy.*t83.*(t37.*t38.*(2.0./3.0)+t37.*t43.*(3.2e+1./1.5e+1)-t26.*(t28-v_min).*2.0+t27.*(t28-v_min).*(8.0./3.0)-t26.*t42.*t45.*4.0+t27.*t42.*t45.*(3.2e+1./5.0)-t26.*t42.*t48.*(3.2e+1./5.0)+t27.*t42.*t48.*(3.2e+1./3.0)+t26.*t38.*(t28-v_min).*2.0-t27.*t38.*(t28-v_min).*(1.6e+1./5.0)+t26.*t43.*(t28-v_min).*(1.6e+1./3.0)-t27.*t43.*(t28-v_min).*(6.4e+1./7.0)+t26.*t45.*(t29-w0_min).*(8.0./3.0)-t27.*t45.*(t29-w0_min).*4.0+t26.*t48.*(t29-w0_min).*4.0-t27.*t48.*(t29-w0_min).*(3.2e+1./5.0)-t37.*t42.*(t29-w0_min).*(1.2e+1./5.0)-t26.*t42.*(t28-v_min).*(t29-w0_min).*(3.2e+1./5.0)+t27.*t42.*(t28-v_min).*(t29-w0_min).*(3.2e+1./3.0)).*2.0+cx.*t84.*(t26.*t45.*2.0-t27.*t45.*(8.0./3.0)+t26.*t48.*(8.0./3.0)-t27.*t48.*4.0-t37.*t42.*(4.0./3.0)+t37.*t44.*(1.6e+1./7.0)+t37.*(t29-w0_min).*(2.0./3.0)-t37.*(t29-w0_min).^3.*(2.0./5.0)+t26.*(t28-v_min).*(t29-w0_min).*(8.0./3.0)-t27.*(t28-v_min).*(t29-w0_min).*4.0-t26.*t38.*t45.*2.0+t27.*t38.*t45.*(1.6e+1./5.0)-t26.*t38.*t48.*(1.6e+1./5.0)+t27.*t38.*t48.*(1.6e+1./3.0)-t26.*t43.*t45.*(1.6e+1./3.0)+t27.*t43.*t45.*(6.4e+1./7.0)-t26.*t43.*t48.*(6.4e+1./7.0)+t37.*t38.*t42.*(3.2e+1./1.5e+1)+t27.*t43.*t48.*1.6e+1-t26.*t42.*(t28-v_min).*4.0+t27.*t42.*(t28-v_min).*(3.2e+1./5.0)-t37.*t43.*(t29-w0_min).*(8.0e+1./2.1e+1)+t26.*t42.*t45.*(t29-w0_min).*(3.2e+1./5.0)-t27.*t42.*t45.*(t29-w0_min).*(3.2e+1./3.0)+t26.*t42.*t48.*(t29-w0_min).*(3.2e+1./3.0)-t27.*t42.*t48.*(t29-w0_min).*(1.28e+2./7.0)).*2.0,cy.*t83.*(t25.*t42.*(8.0./3.0)-t25.*t44.*(6.4e+1./2.1e+1)-t25.*(t29-w0_min).*2.0+t25.*(t29-w0_min).^3.*(2.0./3.0)-t25.*t38.*t42.*(1.6e+1./5.0)+t25.*t43.*(t29-w0_min).*(1.6e+1./3.0)).*2.0+cx.*t84.*(-v_min+v_max+t25.*t38.*(4.0./3.0)+t25.*t43.*(1.6e+1./5.0)-t25.*t42.*(t29-w0_min).*4.0).*2.0];