order = 3;
fs  = 22050;
fhp = 200;
flp = 3500;

% [B, A] = butter(order, 2*fhp/fs, 'high');
% [B, A] = butter(order, 2*flp/fs, 'low');
[B, A] = butter(order, 2*[fhp flp]/fs);

fprintf('const double b_coefficients[] = {');
for i=1:length(B)-1
  fprintf('%f, ', B(i));
end
fprintf('%f', B(end));
fprintf('};\n');

fprintf('const double a_coefficients[] = {');
for i=1:length(A)-1
  fprintf('%f, ', A(i));
end
fprintf('%f', A(end));
fprintf('};\n');
