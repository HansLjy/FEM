syms A B C D E F real

% vertex-face & edge-edge, their forms are the same

delta = A * C - B * B;
lambda3 = (C * D - B * E) / delta;
lambda2 = (-B * D + A * E) / delta;

s_vf = F + lambda3^2 * A + lambda2^2 * C - 2 * lambda3 * D - 2 * lambda2 * E + 2 * lambda2 * lambda3 * B;
s_ee = F + lambda3^2 * A + lambda2^2 * C - 2 * lambda3 * D + 2 * lambda2 * E - 2 * lambda2 * lambda3 * B;

gradient_vf = simplify(gradient(s_vf, [A, B, C, D, E, F]));
hessian_vf = simplify(hessian(s_vf, [A, B, C, D, E, F]));

gradient_ee = simplify(gradient(s_ee, [A, B, C, D, E, F]));
hessian_ee = simplify(hessian(s_ee, [A, B, C, D, E, F]));

matlabFunction(gradient_vf, 'File', 'genMatlabCode/VFDistanceGradient', 'Vars', [A, B, C, D, E, F], 'Outputs', {'gradient'});
matlabFunction(hessian_vf, 'File', 'genMatlabCode/VFDistanceHessian', 'Vars', [A, B, C, D, E, F], 'Outputs', {'hessian'});

matlabFunction(gradient_ee, 'File', 'genMatlabCode/EEDistanceGradient', 'Vars', [A, B, C, D, E, F], 'Outputs', {'gradient'});
matlabFunction(hessian_ee, 'File', 'genMatlabCode/EEDistanceHessian', 'Vars', [A, B, C, D, E, F], 'Outputs', {'hessian'});


cfg = coder.config('lib');
cfg.DataTypeReplacement = 'CBuiltIn';

codegen('genMatlabCode/VFDistanceGradient.m', '-d', 'genCPPCode/VFDistanceGradient', '-c', '-lang:c++', '-args', {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, '-config:lib')
codegen('genMatlabCode/VFDistanceHessian.m', '-d', 'genCPPCode/VFDistanceHessian', '-c', '-lang:c++', '-args', {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, '-config:lib')
codegen('genMatlabCode/EEDistanceGradient.m', '-d', 'genCPPCode/EEDistanceGradient', '-c', '-lang:c++', '-args', {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, '-config:lib')
codegen('genMatlabCode/EEDistanceHessian.m', '-d', 'genCPPCode/EEDistanceHessian', '-c', '-lang:c++', '-args', {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, '-config:lib')
