syms e_prev [3, 1] real
syms e_next [3, 1] real

t1 = e_prev / sqrt(dot(e_prev, e_prev))
t2 = e_next / sqrt(dot(e_next, e_next))

C = dot(t1, t2);
S = cross(t1, t2);

gradient_C = simplify(gradient(C, [e_prev; e_next]));
hessian_C = simplify(hessian(C, [e_prev; e_next]));

gradient_S = [simplify(gradient(S(1), [e_prev; e_next])) simplify(gradient(S(2), [e_prev; e_next])) simplify(gradient(S(3), [e_prev; e_next]))];

hessian_S = [simplify(hessian(S(1), [e_prev; e_next])) simplify(hessian(S(2), [e_prev; e_next])) simplify(hessian(S(3), [e_prev; e_next]))];

matlabFunction(gradient_C, 'File', 'genMatlabCode/PTCGradient', 'Vars', {e_prev, e_next}, 'Outputs', {'gradient'});
matlabFunction(hessian_C, 'File', 'genMatlabCode/PTCHessian', 'Vars', {e_prev, e_next}, 'Outputs', {'hessian'});

matlabFunction(gradient_S, 'File', 'genMatlabCode/PTSGradient', 'Vars', {e_prev, e_next}, 'Outputs', {'gradient'});
matlabFunction(hessian_S, 'File', 'genMatlabCode/PTSHessian', 'Vars', {e_prev, e_next}, 'Outputs', {'hessian'});

cfg = coder.config('lib');
cfg.DataTypeReplacement = 'CBuiltIn';

vec3 = [1.0; 1.0; 1.0]

codegen('genMatlabCode/PTCGradient.m', '-d', 'genCPPCode/PTCGradient', '-c', '-lang:c++', '-args', {vec3, vec3}, '-config:lib')
codegen('genMatlabCode/PTCHessian.m', '-d', 'genCPPCode/PTCHessian', '-c', '-lang:c++', '-args', {vec3, vec3}, '-config:lib')
codegen('genMatlabCode/PTSGradient.m', '-d', 'genCPPCode/PTSGradient', '-c', '-lang:c++', '-args', {vec3, vec3}, '-config:lib')
codegen('genMatlabCode/PTSHessian.m', '-d', 'genCPPCode/PTSHessian', '-c', '-lang:c++', '-args', {vec3, vec3}, '-config:lib')
