from rocketpy import Environment, SolidMotor, Rocket, Flight
import datetime

tomorrow = datetime.date.today() + datetime.timedelta(days=1)

def example():
    env = Environment(latitude=32.990254, longitude=-106.974998, elevation=1400)
    env.set_date(
        (tomorrow.year, tomorrow.month, tomorrow.day, 12)
    )  # Hour given in UTC time
    env.set_atmospheric_model(type="Forecast", file="GFS")
    env.info()

def example_edinburgh():
    env = Environment(
        latitude=55.9533,
        longitude=-3.1883,
        elevation=100
    )
    env.set_date(
        (tomorrow.year, tomorrow.month, tomorrow.day, 12)
    )  # Hour given in UTC time
    env.set_atmospheric_model(type="Forecast", file="GFS")

    altitude = 1500  # meters
    wind_speed = env.wind_speed(altitude)
    
    print(f"Wind speed at {altitude} m: {wind_speed} m/s")



if __name__=="__main__":
    example_edinburgh()