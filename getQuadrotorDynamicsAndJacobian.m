function getQuadrotorDynamicsAndJacobian()
    
    syms x [12 1] real
    syms u [4 1] real
    mass = 1.2;
    Ib = diag([1.2416, 1.2416, 2 * 1.2416]);

    
    eta_b = x(4:6);
    dot_eta_b = x(10:12);
    u_T = u(1);
    tau_b = u(2:4);

    %Matrici
    Q = [1, 0, -sin(eta_b(2));
         0, cos(eta_b(1)), cos(eta_b(2))*sin(eta_b(1));
         0, -sin(eta_b(1)), cos(eta_b(1))*cos(eta_b(2))];

    Q_dot = [0, 0, -cos(eta_b(2))*dot_eta_b(2);
             0, -sin(eta_b(1))*dot_eta_b(1), -sin(eta_b(2))*sin(eta_b(1))*dot_eta_b(2)+cos(eta_b(2))*cos(eta_b(1))*dot_eta_b(1);
             0, -cos(eta_b(1))*dot_eta_b(1), -sin(eta_b(2))*cos(eta_b(1))*dot_eta_b(2)-cos(eta_b(2))*sin(eta_b(1))*dot_eta_b(1)];

    M = Q' * Ib * Q;
    S = skew(Q * dot_eta_b);
    C = Q' * S * Ib * Q + Q' * Ib * Q_dot;

    Rb = [cos(eta_b(2))*cos(eta_b(3)), sin(eta_b(1))*sin(eta_b(2))*cos(eta_b(3))-cos(eta_b(1))*sin(eta_b(3)), cos(eta_b(1))*sin(eta_b(2))*cos(eta_b(3))+sin(eta_b(1))*sin(eta_b(3));
          cos(eta_b(2))*sin(eta_b(3)), sin(eta_b(1))*sin(eta_b(2))*sin(eta_b(3))+cos(eta_b(1))*cos(eta_b(3)), cos(eta_b(1))*sin(eta_b(2))*sin(eta_b(3))-sin(eta_b(1))*cos(eta_b(3));
         -sin(eta_b(2)),              sin(eta_b(1))*cos(eta_b(2)),                                          cos(eta_b(1))*cos(eta_b(2))];

    %Dinamica
    lin_acc = 9.81*[0; 0; 1] - (u_T/mass)*Rb*[0; 0; 1];
    ddot_eta_b = (inv(M))*((-C*dot_eta_b)+(Q'*tau_b));

    dx = sym('dx', [12 1], 'real');
    dx(1:3) = x(7:9);
    dx(4:6) = dot_eta_b;
    dx(7:9) = lin_acc;
    dx(10:12) = ddot_eta_b;

    %Jacobian
    A = jacobian(dx, x);
    B = jacobian(dx, u);

    
    matlabFunction(dx, 'Vars', {x, u}, 'File', 'QuadrotorStateFcn', 'Optimize', true);
    matlabFunction(A, B, 'Vars', {x, u}, 'File', 'QuadrotorStateJacobianFcn', 'Optimize', true);
end

function S = skew(v)
    S = [   0    -v(3)   v(2);
          v(3)     0   -v(1);
         -v(2)   v(1)     0 ];
end