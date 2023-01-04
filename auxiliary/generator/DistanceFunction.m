syms A B C D E F real

% vertex-face & edge-edge, their forms are the same

delta = A * C - B * B;
lambda3 = (C * D - B * E) / delta;
lambda2 = (-B * D + A * E) / delta;

s = F + lambda3^2 * A + lambda2^2 * C - 2 * lambda3 * D - 2 * lambda2 * E + 2 * lambda2 * lambda3 * B;

disp('Vertex-Face or Edge-Edge case');
gradient = simplify(gradient(s, [A, B, C, D, E, F]));
hessian = simplify(hessian(s, [A, B, C, D, E, F]));

matlabFunction(gradient, hessian, 'File', 'BarrierFunction', 'Vars', [A, B, C, D, E, F], 'Outputs', {'gradient', 'hessian'});

% vertex-line

% s2 = delta / C;

% disp('Vertex-Line:')
% latex(simplify(gradient(s2, [A, B, C])))
% latex(simplify(hessian(s2, [A, B, C])))

% matlabFunction()