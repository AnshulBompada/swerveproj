package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class SwerveMod {
    double global_rotation;
    WPI_TalonSRX m_speed;
    WPI_TalonSRX m_rotation;
    PWMSparkMax sim_speed;
    PWMSparkMax sim_rotation;

    public SwerveMod(WPI_TalonSRX speed, WPI_TalonSRX rotation) {
        m_speed = speed;
        m_rotation = rotation;
    }

    public SwerveMod(PWMSparkMax speed, PWMSparkMax rotation) {
      sim_speed = speed;
      sim_rotation = rotation;
    }


    public void control(double speed, double rotation) {
      turntoang(rotation * 360);
      m_speed.set(speed);
  }


  public void turntoang(double rotationdegrees) {
      double time = (75 / (13*180))/rotationdegrees - (75 / (13*180))* global_rotation;
      double speed =1;
      if(time < 0) {
        time = -time;
        speed = -speed;
      }
      turntime(m_rotation, speed, time);
      global_rotation =+ rotationdegrees;
  }
  
    public void turntime(WPI_TalonSRX motor, double speed, double time){
      Timer timer = new Timer();
      timer.start();
      while(timer.get() < time) {
        motor.set(speed);
  }
      motor.set(0);
  }

  public void turntime(PWMSparkMax motor, double speed, double time){
    Timer timer = new Timer();
    timer.start();
    while(timer.get() < time) {
      motor.set(speed);
}
    motor.set(0);
}
}