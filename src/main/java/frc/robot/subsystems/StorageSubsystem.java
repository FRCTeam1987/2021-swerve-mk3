package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.DigitalDebouncer;

public class StorageSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public final WPI_TalonSRX topTalon;
  public final WPI_TalonSRX bottomTalon;
  public final DigitalInput ballSensor;
  private final DigitalDebouncer debouncer;
  private int ballsInStorage;

  public StorageSubsystem() {
    topTalon = new WPI_TalonSRX(Constants.STORAGE_TOP_MOTOR);
    bottomTalon = new WPI_TalonSRX(Constants.STORAGE_BOTTOM_MOTOR);
    ballSensor = new DigitalInput(Constants.STORAGE_BALL_SENSOR);
    debouncer = new DigitalDebouncer(Constants.STORAGE_BALL_TIME_DEBOUNCER);
    ballsInStorage = Constants.StorageSubsystem.startingBallCount;
  }

  public void setNumOfBallsInStorage(int numOfBalls){
    ballsInStorage = numOfBalls;
  }

  public void incrementNumofBalls(){
    ballsInStorage++;
  }

  public void decrementNumofBalls(){
    ballsInStorage--;
  }

  public int getNumOfBalls(){
    return ballsInStorage;
  }

  public boolean isBallAtEntrance(){
    return debouncer.get();
  }

  public boolean hasMaxNumberBalls(){
    return getNumOfBalls() >= Constants.StorageSubsystem.maxBallCount;
  }

  public void stop() {
    topTalon.set(0);
    bottomTalon.set(0);
  }

  public void spinmotorTop(double spinSpeed) {
    topTalon.set(spinSpeed);
  }

  public void spinmotorBottom(double spinSpeed) {
    bottomTalon.set(spinSpeed);
  }

  @Override
  public void periodic() {
    debouncer.periodic(ballSensor.get());
    SmartDashboard.putNumber("ball count", getNumOfBalls());
  }
}
















