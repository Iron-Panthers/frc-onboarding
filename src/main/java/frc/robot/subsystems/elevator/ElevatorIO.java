package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean leftMotorConnected = true;
    public boolean rightMotorConnected = true;

    public double leftPositionRads = 0;
    public double leftVelocityRPM = 0;
    public double leftAppliedVolts = 0;
    public double leftSupplyCurrent = 0;
    public double leftTempCelcius = 0;

    public double rightPositionRads = 0;
    public double rightVelocityRPM = 0;
    public double rightAppliedVolts = 0;
    public double rightSupplyCurrent = 0;
    public double rightTempCelcius = 0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  /* Run left and right motors at target velocities (RPM) */
  default void runVelocity(int leftRPM, int rightRPM) {}

  /* set slot0 (PID + ff) for both motors */
  default void setSlot0(double kP, double kI, double kD, double kS, double kV, double kA) {}

  default void stop() {}
}
