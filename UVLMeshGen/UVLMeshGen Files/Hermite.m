function [H] = Hermite (x, xi, xj)

h = xj - xi;

H.h1 = 1 - 3*(x - xi)^2 / (h^2) + 2*(x - xi)^3/(h^3);
H.h2 = (x - xi) - 2/h*(x - xi)^2 + 1/(h^2)*(x - xi)^3;
H.h3 = 3/h^2*(x - xi)^2 - 2/(h^3)*(x - xi)^3;
H.h4 = -1/h*(x - xi)^2 + 1/(h^2)*(x - xi)^3;


end

