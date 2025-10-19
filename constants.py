import numpy as np


def eng_to_numpy(fp:str) -> np.ndarray:
    """Converts a file in eng format to a numpy array of data"""
    with open(fp, "r") as file:
        value_strings = file.read().split("\n")[1:-2] # extracts only strings of data from all metadata
        data_arr = np.array([val.split(" ") for val in value_strings]).astype(np.float64)
        return data_arr
    raise(RuntimeError("Unable to open or parse file."))


# thrust profile for Cesaroni motor
def thrust_profile_cesaroni(t): 
    if t>2.15: return 0.0
    if t>=0.0: return np.interp(t,cesaroni_data[:,0], cesaroni_data[:,1])
    else: raise ValueError("t value can't be negative.")


g = 9.81  # acceleration due to gravity in m/s^2
rho0 =  1.225  # density of air at seas level in kg/m^3
cesaroni_data = eng_to_numpy(r"C:\Users\rizzo\Code\VSC-Asus\University Code\Endeavour\Bayes-Simulator-main\data\Cesaroni_266H125-12A.eng")

# Rocket parameters
total_mass = 0.923
fuel_mass = 0.125
burn_rate = 0.125/2.15
length = 1.19
radius = 0.0675/2.
moi_yaw_pitch = total_mass*((1/4)*radius**2 + (1/3)*length**2)
moi_roll = total_mass*(radius**2)/2

params = {
    'dry_mass': total_mass-fuel_mass, # mass of rocket in KG 
    'fuel_initial': fuel_mass, # initial fuel mass in KG
    'burn_rate': burn_rate,
    # for cylinder roll axis, MOI = 0.5*MR^2. for yaw/pitch (1/4)​*MR2+(1/3)*​ML2 where L is cylinder length
    'I_body': np.diag([moi_yaw_pitch, moi_yaw_pitch, moi_roll]), # moment of inertia of the body in kg*m^2
    'Cd_body': 0.375,
    'Num_cannards': 4, # number of canards
    'A_ref': np.pi * (radius)**2,
    'A_canard': 0.03*0.02,
    'thrust': thrust_profile_cesaroni,
    'canard_angles': np.radians([0, 0, 0, 0]), # angles of canards in radians, start by thinking about a static case
    'canard_vertical_position': 0.1, # vertical position of canards in meters
    'rocket_radius': radius, # radius of the rocket in meters
    'CP_R':[0.106, 0, 0] # Distance between the center of pressure and the center of mass in meters
}




if __name__=="__main__":
    print(cesaroni_data)
    print(cesaroni_data[:,0])