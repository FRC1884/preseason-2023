package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
  private static Elevator instance;

  public static Elevator getInstance() {
    if (instance == null) instance = new Elevator();
    return instance;
  }

  // Motors
  private CANSparkMax leftMotor, rightMotor;

  // PID Controller
  private SparkMaxPIDController leftMotorController, rightMotorController;

  // Gains
  private double kP = 0;
  private double kI = 0;
  private double kD = 0;
  private double kIz = 0;
  private double kFF = 0;

  public static enum SetPoint {
    BOTTOM(23),
    MIDDLE(23),
    TOP(21);

    int pos;

    SetPoint(int pos) {
      this.pos = pos;
    }
  }

  public static SetPoint currentState;

  private Elevator() {

    leftMotor = new CANSparkMax(RobotMap.ElevatorMap.leader, MotorType.kBrushless);
    rightMotor = new CANSparkMax(RobotMap.ElevatorMap.follow, MotorType.kBrushless);

    leftMotorController = leftMotor.getPIDController();
    // rightMotorController = rightMotor.getPIDController();

    // rightMotor.setInverted(true);
    rightMotor.follow(leftMotor, true); // Inverted

    configPID(leftMotorController);
    // configPID(rightMotorController);

    leftMotor.clearFaults();
  }

  private void configPID(SparkMaxPIDController controller) {
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
  }

  public boolean setPosition(int position) {
    leftMotorController.setReference(position, ControlType.kPosition);
    return true;
  }

  public void setPosition(SetPoint setPoint) {
    if (setPosition(setPoint.pos)) {
      currentState = setPoint;
      // return true;
    }
    // return false;

  }

  public FunctionalCommand moveElevatorCommand(
      SetPoint setPoint) { // FIXME If elevator is switched kill motor
    return new FunctionalCommand(
        () -> setPosition(setPoint),
        () -> {},
        interrupted -> leftMotor.stopMotor(),
        () -> false,
        this);
  }
}
