package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean topMotorConnected = true;
    public boolean bottomMotorConnected = true;

    public double topPositionRads = 0;
    public double topVelocityRPM = 0;
    public double topAppliedVolts = 0;
    public double topSupplyCurrent = 0;
    public double topTempCelcius = 0;

    public double bottomPositionRads = 0;
    public double bottomVelocityRPM = 0;
    public double bottomAppliedVolts = 0;
    public double bottomSupplyCurrent = 0;
    public double bottomTempCelcius = 0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  /* Run top and bottom flywheels at target velocities (RPM) */
  default void runVelocity(int topRPM, int bottomRPM) {}

  /* set slot0 (PID + ff) for both motors */
  default void setSlot0(double kP, double kI, double kD, double kS, double kV, double kA) {}

  default void stop() {}
}
