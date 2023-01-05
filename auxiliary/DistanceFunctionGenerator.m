syms A B C D E F real

% vertex-face & edge-edge, their forms are the same

delta = A * C - B * B;
lambda3 = (C * D - B * E) / delta;
lambda2 = (-B * D + A * E) / delta;

s = F + lambda3^2 * A + lambda2^2 * C - 2 * lambda3 * D - 2 * lambda2 * E + 2 * lambda2 * lambda3 * B;

disp('Vertex-Face or Edge-Edge case');
gradient = simplify(gradient(s, [A, B, C, D, E, F]));
hessian = simplify(hessian(s, [A, B, C, D, E, F]));

matlabFunction(gradient, 'File', 'genMatlabCode/DistanceGradient', 'Vars', [A, B, C, D, E, F], 'Outputs', {'gradient'});
matlabFunction(hessian, 'File', 'genMatlabCode/DistanceHessian', 'Vars', [A, B, C, D, E, F], 'Outputs', {'hessian'});

cfg = coder.config('lib');
cfg.DataTypeReplacement = 'CBuiltIn';

codegen('genMatlabCode/DistanceGradient.m', '-d', 'genCPPCode/DistanceGradient', '-c', '-lang:c++', '-args', {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, '-config:lib')
codegen('genMatlabCode/DistanceHessian.m', '-d', 'genCPPCode/DistanceHessian', '-c', '-lang:c++', '-args', {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, '-config:lib')
