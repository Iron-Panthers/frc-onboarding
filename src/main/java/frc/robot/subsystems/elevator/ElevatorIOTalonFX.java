package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX leftTalon;
  private final TalonFX rightTalon;

  private final StatusSignal<Double> leftPositionRads;
  private final StatusSignal<Double> leftVelocityRPM;
  private final StatusSignal<Double> leftAppliedVolts;
  private final StatusSignal<Double> leftSupplyCurrent;
  private final StatusSignal<Double> leftTemp;
  private final StatusSignal<Double> rightPositionRads;
  private final StatusSignal<Double> rightVelocityRPM;
  private final StatusSignal<Double> rightAppliedVolts;
  private final StatusSignal<Double> rightSupplyCurrent;
  private final StatusSignal<Double> rightTemp;

  private final Slot0Configs gainsConfig = new Slot0Configs();
  private final VelocityVoltage velocityOutput = new VelocityVoltage(0).withUpdateFreqHz(0);
  private final NeutralOut neutralOutput = new NeutralOut();

  public ElevatorIOTalonFX() {
    leftTalon = new TalonFX(ELEVATOR_CONFIG.leftID());
    rightTalon = new TalonFX(ELEVATOR_CONFIG.rightID());

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 60.0; // FIXME
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = ELEVATOR_CONFIG.reduction();

    gainsConfig.kP = GAINS.kP();
    gainsConfig.kI = GAINS.kI();
    gainsConfig.kD = GAINS.kD();
    gainsConfig.kS = GAINS.kS();
    gainsConfig.kV = GAINS.kV();
    gainsConfig.kA = GAINS.kA();

    leftTalon.getConfigurator().apply(config);
    rightTalon.getConfigurator().apply(config);
    leftTalon.getConfigurator().apply(gainsConfig);
    rightTalon.getConfigurator().apply(gainsConfig);

    leftTalon.setInverted(true);
    rightTalon.setInverted(true); // FIXME

    leftPositionRads = leftTalon.getPosition();
    leftVelocityRPM = leftTalon.getVelocity();
    leftAppliedVolts = leftTalon.getMotorVoltage();
    leftSupplyCurrent = leftTalon.getSupplyCurrent();
    leftTemp = leftTalon.getDeviceTemp();

    rightPositionRads = rightTalon.getPosition();
    rightVelocityRPM = rightTalon.getVelocity();
    rightAppliedVolts = rightTalon.getMotorVoltage();
    rightSupplyCurrent = rightTalon.getSupplyCurrent();
    rightTemp = rightTalon.getDeviceTemp();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftMotorConnected =
        BaseStatusSignal.refreshAll(
                leftPositionRads, leftVelocityRPM, leftAppliedVolts, leftSupplyCurrent, leftTemp)
            .isOK();
    inputs.rightMotorConnected =
        BaseStatusSignal.refreshAll(
                rightPositionRads,
                rightVelocityRPM,
                rightAppliedVolts,
                rightSupplyCurrent,
                rightTemp)
            .isOK();

    inputs.leftPositionRads = Units.rotationsToRadians(leftPositionRads.getValueAsDouble());
    inputs.leftVelocityRPM = leftVelocityRPM.getValueAsDouble() * 60.0;
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftSupplyCurrent = leftSupplyCurrent.getValueAsDouble();
    inputs.leftTempCelcius = leftTemp.getValueAsDouble();

    inputs.rightPositionRads = Units.rotationsToRadians(rightPositionRads.getValueAsDouble());
    inputs.rightVelocityRPM = rightVelocityRPM.getValueAsDouble() * 60.0;
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightSupplyCurrent = rightSupplyCurrent.getValueAsDouble();
    inputs.rightTempCelcius = rightTemp.getValueAsDouble();
  }

  @Override
  public void runVelocity(int leftVelocity, int rightVelocity) {
    leftTalon.setControl(velocityOutput.withVelocity(leftVelocity / 60));
    rightTalon.setControl(velocityOutput.withVelocity(rightVelocity / 60));
  }

  @Override
  public void setSlot0(double kP, double kI, double kD, double kS, double kV, double kA) {
    gainsConfig.kP = kP;
    gainsConfig.kI = kI;
    gainsConfig.kD = kD;
    gainsConfig.kS = kS;
    gainsConfig.kV = kV;
    gainsConfig.kA = kA;

    leftTalon.getConfigurator().apply(gainsConfig);
    rightTalon.getConfigurator().apply(gainsConfig);
  }

  @Override
  public void stop() {
    leftTalon.setControl(neutralOutput);
    rightTalon.setControl(neutralOutput);
  }
}
