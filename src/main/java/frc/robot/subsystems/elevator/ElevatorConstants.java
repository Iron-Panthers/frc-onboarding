package frc.robot.subsystems.elevator;

import frc.robot.Constants;

public class ElevatorConstants {
  public static final ElevatorConfig ELEVATOR_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new ElevatorConfig(15, 16, 0.75); // FIXME
        case DEV -> new ElevatorConfig(0, 0, 1);
        case SIM -> new ElevatorConfig(0, 0, 1);
      };

  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case COMP -> new PIDGains(0, 0, 0, 0, 0.09, 0); // FIXME
        case DEV -> new PIDGains(0, 0, 0, 0, 0, 0);
        case SIM -> new PIDGains(0, 0, 0, 0, 0, 0);
      };

  public record ElevatorConfig(int topID, int bottomID, double reduction) {}

  public record PIDGains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
