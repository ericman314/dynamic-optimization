def Fg(h, M = 5.972e24, r_planet=6371393):
    # Calculates the gravitational acceleration
    # mass earth = 5.972e24kg, radius = 6371393m
    d = h + r_planet  # h = distance (m) from surface
    G = 6.67408e-11   # m3kg-1s-2 (gravitational const.)
    return G*M/(d**2)


def air_dens(h, g = 9.80665, Po = 101325, To = 288.15):
    # See https://en.wikipedia.org/wiki/Density_of_air (Variation with altitude)
    # See http://www.atmo.arizona.edu/students/courselinks/fall14/atmo336/lectures/sec1/structure.html for Temp
    L = 0.0065     # K/m (Temperature Lapse Rate)
    R = 8.31447    # J/mol*K (Universal gas constant)
    M = 0.0289644  # kg/mol (Molar mass of dry air), will change w/humidity
    T, P = To, Po         # Initiate Variable
    if h < 18*1000:
        P = Po*(1-L*h/To)**(g*M/(R*L))  # Pressure (Pa)
        T = To - L * h  # Temperature (K)
        if T <= 218.5:
            T = 218.5
    elif h >= 18*1000:  # Above pressure correlation only good for troposphere
        n = (h-18*1000)/5500
        P = 7150*(0.5**n)   # Pressure drops in half every 5.5km, empirical correlation
        T = 218.15          # Above Troposphere, temp gets strange, may need to fix this
    rho = P*M/(R*T)  # Density Dry Air
    return P, T, rho



