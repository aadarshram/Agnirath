import numpy as np
import matplotlib.pyplot as plt


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

    def calculate_power(self, speed):
        # Calculate power required to overcome rolling resistance and aerodynamic drag
        self.dynamic_speed_crr = (self.no_of_wheels / 3) * 4.1 * 10 ** (-5) * speed
        rolling_resistance = (
            self.mass * 9.8 * (self.zero_speed_crr + self.dynamic_speed_crr)
        )  # Assume coefficient of friction = 0.01
        drag_force = (
            self.frontal_area * 0.5 * self.aerodynamic_coef * 1.225 * speed**2
        )  # Air density = 1.225 kg/m^3
        power = (rolling_resistance + drag_force) * speed
        return power


class ElectricCar:
    def __init__(self, motor, distance, battery_capacity):
        self.motor = motor
        self.dt = 1  # seconds

        self.distance = distance  # meters
        self.time_elapsed = 0  # seconds
        self.distance_traveled = 0  # meters
        self.energy_consumed_car = 0  # Wh

        # Battery
        self.remaining_energy = battery_capacity

    def drive(self, start_speed, acceleration, remaining_energy):
        self.acceleration = acceleration  # m/s^2
        self.speed = start_speed
        self.remaining_energy = remaining_energy
        self.energy_consumed_car = 0
        self.distance_traveled = 0  # m/s
        self.time_elapsed = 0  # seconds
        while self.remaining_energy > 1:
            if self.speed < 30:
                self.speed += self.acceleration * self.dt
            elif self.speed >= 30:
                self.speed = 30
            self.power = self.motor.calculate_power(self.speed)
            self.time_elapsed += self.dt
            self.energy = self.power * self.dt / 3600
            self.instantaneous_distance = self.speed * self.dt
            self.energy_consumed_car += self.energy
            self.remaining_energy -= self.energy
            self.distance_traveled += self.instantaneous_distance
        # print(
        #     f"Time: {self.time_elapsed} seconds, Distance: {self.distance_traveled:.2f} meters, Speed:{self.speed:.3f} m/s, Acceleration:{self.acceleration:.3f} m/s^2, Energy Remaining: {self.remaining_energy:.3f} Wh"
        # )

        return [
            self.time_elapsed,
            self.distance_traveled,
            self.speed,
            self.acceleration,
            self.remaining_energy,
        ]

    def drive_eval(self):
        acceleration = 0.1
        drive_results = []
        while acceleration <= 2:
            # print(acceleration)
            drive_details = self.drive(0, acceleration,5000)
            drive_results.append(drive_details)
            acceleration += 0.1

        for drive_details in drive_results:
            print(
                f"Time: {drive_details[0]} s, Distance: {drive_details[1]:.2f} m, Speed:{drive_details[2]:.3f} m/s, Acceleration: {drive_details[3]:.1f} m/s^2, Energy Remaining: {drive_details[4]:.3f} Wh"
            )


# Main program


def main():
    distance = 3000000  # meters
    battery_capacity = 5000
    wheel_radius = 19 / 39.37  # Convert inches to meters
    mass = 314  # kg
    wheels = 3
    aerodynamic_coef = 0.12
    frontal_area = 1
    zero_speed_crr = 0.003

    motor = Motor(
        wheel_radius, mass, wheels, aerodynamic_coef, frontal_area, zero_speed_crr
    )
    car = ElectricCar(motor, distance, battery_capacity)

    car.drive_eval()


if __name__ == "__main__":
    main()
