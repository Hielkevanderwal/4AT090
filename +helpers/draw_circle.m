function draw_circle(a,b,r,style)
% Plot fitted circle 
theta = linspace(0, 2*pi, 200);
x_circle = a + r*cos(theta);
y_circle = b + r*sin(theta);
plot(x_circle, y_circle, style , 'LineWidth', 2, 'DisplayName', 'Fitted Circle');
end