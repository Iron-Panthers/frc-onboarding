package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // RPM
  public enum VelocityTarget {
    IDLE(0, 0),
    SHOOT(7500, 7500),
    SLOW(1200, 1200);
    private int topVelocity, bottomVelocity;

    private VelocityTarget(int topVelocity, int bottomVelocity) {
      this.topVelocity = topVelocity;
      this.bottomVelocity = bottomVelocity;
    }
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  @AutoLogOutput(key = "Mechanism/Elevator/Target")
  private VelocityTarget velocityTarget = VelocityTarget.IDLE;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Mechanism/Elevator", inputs);

    io.runVelocity(velocityTarget.topVelocity, velocityTarget.bottomVelocity);

    Logger.recordOutput("Mechanism/Elevator/TopTargetVelocity", velocityTarget.topVelocity);
    Logger.recordOutput("Mechanism/Elevator/BottomTargetVelocity", velocityTarget.bottomVelocity);
  }

  public void setVelocityTarget(VelocityTarget target) {
    velocityTarget = target;
  }
}
