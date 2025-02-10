def inertia(mass, var_1, var_2):
    factor = 1.0/12.0
    mass_factor = mass * factor
    var_sq1 = var_1**2
    var_sq2 = var_2**2

    return mass_factor * (var_sq1 + var_sq2)

var = inertia(0.018, 0.034, 0.034)
print(var)
