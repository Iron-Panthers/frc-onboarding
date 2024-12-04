package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // RPM
  public enum ElevatorVelocityTarget {
    IDLE(0, 0),
    UP(7500, 7500),
    DOWN(1200, 1200);
    private int leftVelocity, rightVelocity;

    private ElevatorVelocityTarget(int leftVelocity, int rightVelocity) {
      this.leftVelocity = leftVelocity;
      this.rightVelocity = rightVelocity;
    }
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  @AutoLogOutput(key = "Mechanism/Elevator/Target")
  private ElevatorVelocityTarget velocityTarget = ElevatorVelocityTarget.IDLE;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Mechanism/Elevator", inputs);

    io.runVelocity(velocityTarget.leftVelocity, velocityTarget.rightVelocity);

    Logger.recordOutput("Mechanism/Elevator/leftTargetVelocity", velocityTarget.leftVelocity);
    Logger.recordOutput("Mechanism/Elevator/rightTargetVelocity", velocityTarget.rightVelocity);
  }

  public void setVelocityTarget(ElevatorVelocityTarget target) {
    velocityTarget = target;
  }
}
