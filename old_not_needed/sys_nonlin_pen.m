function [dx,y] = sys_nonlin_pen(t,x,u,r_p,d_p,m_p,g,varargin)
% th1dd = -(damp_pendulum*th1d + g*m_p*r_p*sin(th1))/(m_p*r_p^2)
% x = [th1d; th1];
    y = x(2);
    dx = [-(d_p*x(1) + g*m_p*r_p*sin(x(2)))/(m_p*r_p^2);
          x(1)];
end