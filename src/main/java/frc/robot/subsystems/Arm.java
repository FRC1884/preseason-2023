package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {
  private static Arm instance;

  public static Arm getInstance() {
    if (instance == null) instance = new Arm();
    return instance;
  }

  // Motors
  private CANSparkMax motor, rotater;

  // PID Controller
  private SparkMaxPIDController motorController, rotaterController;

  // Gains
  private double kP = 0;
  private double kI = 0;
  private double kD = 0;
  private double kIz = 0;
  private double kFF = 0;

  public static enum ArmState {
    BOTTOM(23),
    MIDDLE(23),
    TOP(21);

    int pos;

    ArmState(int pos) {
      this.pos = pos;
    }
  }

  public static ArmState currentState;

  private Arm() {

    motor = new CANSparkMax(RobotMap.ArmMap.MotorMaster, MotorType.kBrushless);
    rotater = new CANSparkMax(RobotMap.ArmMap.rotatermaster, MotorType.kBrushless);

    motorController = motor.getPIDController();
    rotaterController = rotater.getPIDController();

    configPID(motorController);
    configPID(rotaterController);

    motor.clearFaults();
    rotater.clearFaults();
  }

  private void configPID(SparkMaxPIDController controller) {
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
  }

  public boolean rotaterSetPosition(int position) {
    motorController.setReference(position, ControlType.kPosition);
    return true;
  }

  public void rotaterSetPosition(ArmState armstate) {
    if (rotaterSetPosition(armstate.pos)) {
      currentState = armstate;
      // return true;
    }
    // return false;

  }

  public void setMotorSpeed(double speed) {
    motor.set(speed);
  }

  public FunctionalCommand RotateArmCommand(ArmState state) {
    return new FunctionalCommand(
        () -> rotaterSetPosition(state),
        () -> {},
        interrupted -> rotater.stopMotor(),
        () -> false,
        this);
  }
}
