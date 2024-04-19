import numpy as np

class Battery:
    def __init__(self, capacity):
        self.capacity = capacity  # Wh
        self.remaining_energy = capacity  # Wh

    def discharge(self, power, time):
        energy_consumed = power * time 
        if energy_consumed <= self.remaining_energy:
            self.remaining_energy -= energy_consumed
            return energy_consumed
        else:
            energy_consumed = self.remaining_energy
            self.remaining_energy = 0
            return energy_consumed

class Motor:
    def __init__(self, wheel_radius, mass,wheels, aerodynamic_coef,frontal_area,zero_speed_crr):
        self.wheel_radius = wheel_radius  # inches
        self.mass = mass  # kg
        self.aerodynamic_coef = aerodynamic_coef
        self.frontal_area = frontal_area # m^2
        self.zero_speed_crr = zero_speed_crr # 0.003
        self.no_of_wheels = wheels

    def calculate_power(self, speed):
        # Calculate power required to overcome rolling resistance and aerodynamic drag
        self.dynamic_speed_crr = (self.no_of_wheels/3)*(4.1*10**(-5)*speed)
        rolling_resistance = self.mass * 9.8 * (self.zero_speed_crr+self.dynamic_speed_crr)  # Assume coefficient of friction = 0.01
        drag_force = self.frontal_area*0.5 * self.aerodynamic_coef * 1.225 * speed**2  # Air density = 1.225 kg/m^3
        power = (rolling_resistance + drag_force) * speed *  100/98
        return power

class ElectricCar:
    def __init__(self, battery, motor):
        self.battery = battery
        self.motor = motor
        self.dt = 1/3600

    def drive(self, speed):
        speed_mps = speed  # Convert speed from km/h to m/s
        power = self.motor.calculate_power(speed_mps)

        time = self.battery.remaining_energy / (power/3600)  # 
        distance_traveled = 0
        energy_consumed = 0

        for t in range(int(time) + 1):
            energy = self.battery.discharge(power, self.dt)
            distance = speed_mps * self.dt*3600
            energy_consumed += energy
            distance_traveled += distance
        print(f"Time: {t} seconds, Distance: {distance_traveled:.2f} meters, Energy consumed: {energy_consumed:.2f} Wh")

# Main program
battery_capacity = 5300
wheel_radius = 19 / 39.37  # Convert inches to meters
mass = 314  # kg
wheels = 3
aerodynamic_coef = 0.12
frontal_area = 1
zero_speed_crr = 0.003


battery = Battery(battery_capacity)
motor = Motor(wheel_radius, mass,wheels,aerodynamic_coef,frontal_area,zero_speed_crr)
car = ElectricCar(battery, motor)

# Example usage
# required_speed = float(input("Enter the speed of the car (m/s): "))

car.drive(21)
