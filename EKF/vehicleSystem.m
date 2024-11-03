function dxdt = vehicleSystem(t, x, Ac, Bc, u)
    dxdt = Ac * x + Bc * u;
end
