import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


class Motor:
    def __init__(
        self, wheel_radius, mass, wheels, aerodynamic_coef, frontal_area, zero_speed_crr
    ):
        self.wheel_radius = wheel_radius  # inches
        self.mass = mass  # kg
        self.aerodynamic_coef = aerodynamic_coef
        self.frontal_area = frontal_area  # m^2
        self.zero_speed_crr = zero_speed_crr  # 0.003
        self.no_of_wheels = wheels

    def calculate_power(self, speed, acceleration, slope):
        # Calculate power required to overcome rolling resistance and aerodynamic drag
        self.dynamic_speed_crr = (self.no_of_wheels / 3) * 4.1 * 10 ** (-5) * speed
        rolling_resistance = (self.mass * 9.8 * (self.zero_speed_crr + self.dynamic_speed_crr))  # Assume coefficient of friction = 0.01
        drag_force = (self.frontal_area * 0.5 * self.aerodynamic_coef * 1.225 * speed**2)  # Air density = 1.225 kg/m^3
        power = (rolling_resistance + drag_force + self.mass*acceleration + self.mass*9.8*np.sin(slope)) * speed
        return power


class ElectricCar:
    def __init__(self, motor, distance, battery_capacity, route):
        self.motor = motor
        self.dt = 1  # seconds
        self.start_speed = 0  # m/s

        self.route = route  # [distance, elevation]
        self.distance = distance  # meters

        # Battery
        self.remaining_energy = battery_capacity

    def drive_sim(
        self, start_speed, acceleration, remaining_energy, distance_to_travel, slope
    ):
        self.acceleration = acceleration  # m/s^2
        self.speed = start_speed
        remaining_energy = remaining_energy
        self.distance_to_travel = distance_to_travel
        self.slope = slope

        self.energy_consumed_car = 0  # Wh
        self.distance_traveled = 0  # m/s
        self.time_elapsed = 0  # seconds

        while self.distance_traveled < distance_to_travel and self.remaining_energy > 1:
            if self.speed < 30:
                self.speed += self.acceleration * self.dt
            elif self.speed >= 30:
                self.speed = 30
            # print("hi")
            self.power = self.motor.calculate_power(self.speed,self.acceleration,self.slope)
            self.time_elapsed += self.dt
            self.energy = self.power * self.dt / 3600
            self.instantaneous_distance = self.speed * self.dt
            self.energy_consumed_car += self.energy
            remaining_energy -= self.energy
            self.distance_traveled += self.instantaneous_distance
        # print(
        #     f"Time: {self.time_elapsed} seconds, Distance: {self.distance_traveled:.2f} meters, Speed:{self.speed:.3f} m/s, Acceleration:{self.acceleration:.3f} m/s^2, Energy Remaining: {remaining_energy:.3f} Wh"
        # )

        return [
            self.time_elapsed,
            self.distance_traveled,
            self.speed,
            self.acceleration,
            self.energy_consumed_car,
            self.remaining_energy,
        ]

    def drive_eval(self, sub_path,start_speed):
        distance = sub_path[0]  # meters
        slope = sub_path[1]  # degrees
        time = sub_path[2]  # seconds
        start_speed = start_speed
        acceleration = 0.01  # m/s^2
        drive_results = []

        while acceleration <= 2:
            # print(acceleration)
            drive_details = self.drive_sim(
                start_speed, acceleration, self.remaining_energy, distance, slope
            )
            start_speed = drive_details[2]
            # print(start_speed)
            drive_results.append(drive_details)
            acceleration += 0.01

        for i in drive_results:
            print(i)
        
        print(time)
        time_filtered_results = []
        for i in range(len(drive_results)):
            if drive_results[i][0] < time:
                time_filtered_results.append(drive_results[i])
                # print(drive_details[i])

        # print("fiter")
        for i in time_filtered_results:
            print(i)

        best_drive = time_filtered_results[0]

        for i in range(len(time_filtered_results)):
            if time_filtered_results[i][4] < best_drive[4]:
                best_drive = time_filtered_results[i]

        self.remaining_energy -= best_drive[4]

        best_drive[5] = self.remaining_energy
        self.start_speed = best_drive[2]
        return best_drive

        # for drive_details in drive_results:
        #     print(
        #         f"Time: {drive_details[0]} s, Distance: {drive_details[1]:.2f} m, Speed:{drive_details[2]:.3f} m/s, Acceleration: {drive_details[3]:.1f} m/s^2, Energy Remaining: {drive_details[4]:.3f} Wh"
        #     )

    def best_drive_path(self):
        self.drive_instructions = []
        for self.sub_path in self.route:
            self.best_drive = self.drive_eval(self.sub_path,self.start_speed)
            self.drive_instructions.append(self.best_drive)
            print("best drive",self.best_drive)

        for self.drive_paths in self.drive_instructions:
            print(
                f"Time: {self.drive_paths[0]} s, Distance: {self.drive_paths[1]:.2f} m, Speed:{self.drive_paths[2]:.3f} m/s, Acceleration: {self.drive_paths[3]:.4f} m/s^2, Energy Used: {self.drive_paths[4]:.3f} Wh, Energy Remaining: {self.drive_paths[5]:.3f} Wh"
            )


# Main program


def main():
    distance = 3000000  # meters
    battery_capacity = 5000  # Wh
    wheel_radius = 19 / 39.37  # Convert inches to meters
    mass = 314  # kg
    wheels = 3
    aerodynamic_coef = 0.12
    frontal_area = 1
    zero_speed_crr = 0.003
    avg_m_per_s = 20
    # route = [[10000, 1*0.015708], [2000, 4*0.015708], [5000, 2*0.015708], [3000, 1*0.015708],[7000, 2*0.015708],[10000, 3*0.015708]]

    # Read the CSV file into a DataFrame
    df = pd.read_csv('route.csv')  # Replace 'output.csv' with the actual CSV file path
    route = df.values.tolist()

    for i in range(len(route)):
        route[i].append(route[i][0] / avg_m_per_s)
    print("")

    motor = Motor(
        wheel_radius, mass, wheels, aerodynamic_coef, frontal_area, zero_speed_crr
    )
    car = ElectricCar(motor, distance, battery_capacity, route)

    car.best_drive_path()


if __name__ == "__main__":
    main()
