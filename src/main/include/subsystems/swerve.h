#pragma once

#include <units/voltage.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/sensors/CANCoder.h>

#include <numbers>

namespace subsystems {

namespace swerve {

namespace constants {
    constexpr double kDRIVE_RATIO = 6.12;

    constexpr units::inch_t kWHEEL_DIAM = 4_in;

    constexpr units::inch_t kWHEEL_CIRC = kWHEEL_DIAM * std::numbers::pi;

    constexpr units::feet_per_second_t kMAX_WHEEL_SPEED = 18.0_fps;

    constexpr int kDRIVE_ENC_RES = 2048;

    /// @brief Converts from Encoder ticks per 100ms to a velocity.
    /// @param vel Encoder ticks per 100ms
    /// @param ratio Gear ratio (input:output)
    /// @param wheel_circ Circumference of the wheel
    /// @return The real tangential velocity of the wheel surface.
    static units::meters_per_second_t falcon_to_velocity(double vel, double ratio, units::meter_t wheel_circ) {
        return vel * 10 / 2048 / ratio * wheel_circ / 1_s;
    }
}

class Module {
public:
    /// @brief Swerve module constructor for the various encoder+motors
    /// @param drive The CAN ID of the drive motor (TalonFX)
    /// @param turn The CAN ID of the turn motor (TalonFX)
    /// @param enc The CAN ID of the CANCoder
    Module(const int, const int, const int);

    /// @brief Returns the heading of the swerve module, where straight ahead is forward
    /// @return The heading in degrees
    units::radian_t get_heading() const;

    units::meters_per_second_t get_velocity() const;

    /// @brief Sets the desired state of the module (velocity of the drive and position of the turn)
    /// @param ref_state The reference state to base on
    /// @param force_angle Whether or not to force the angle position
    void set_desired_goal(const frc::SwerveModuleState&, bool = false);

    /// @brief Resets the position of the drive motor to 0 encoder counts
    void reset_drive_position();

    /// @brief Returns the position of the swerve module
    /// @return Swerve position based on drive encoder in the motor and the absolute encoder
    /// @see get_state
    frc::SwerveModulePosition get_position() const;

    /// @brief Returns the state of the swerve module
    /// @return Swerve state based on drive encoder in the motor and the absolute encoder
    /// @see get_position
    frc::SwerveModuleState get_state() const;

private:
    /// @brief Sets the raw voltage of the motor
    /// @param volts The voltage from -12V to 12V
    void _set_drive_power(const units::volt_t);

    /// @brief Sets the raw voltage of the motor
    /// @param volts The voltage from -12V to 12V
    void _set_turn_power(const units::volt_t);

    frc2::PIDController heading_controller {
        0.0,
        0.0,
        0.0
    };

    frc2::PIDController drive_controller {
        0.0,
        0.0,
        0.0
    };

    std::unique_ptr<ctre::phoenix::motorcontrol::can::WPI_TalonFX> drive_motor;
    std::unique_ptr<ctre::phoenix::motorcontrol::can::WPI_TalonFX> turn_motor;
    std::unique_ptr<ctre::phoenix::sensors::CANCoder> encoder;

}; // class module

} // namespace swerve

} // namespace subsystems
