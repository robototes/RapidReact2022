package frc.team2412.robot.util;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.team2412.robot.Hardware.HardwareConstants;

/**
 * based on https://gist.github.com/SCOTSBots/4daf384311d32e9ff0aad2f179819139
 * the reason why not extend or inherit the ColorSensorV3 is because
 * in order to use any device attached to the Multiplexer have to setChannel() beforehand
 * so if called any method that's not been override will cause trouble
 */
public class MultiplexedColorSensor {
    // The multiplexer I2C is static because it needs to be used for ALL of the multiplexer sensors,
    // and so by making it static all sensors can access it.
    private static I2C i2c_multiplexer;
    // The actual sensor. All of the methods call this sensor to get the data.
    private final ColorSensorV3 sensor;
    // What port on the multiplexer the color sensor is plugged into.
    private final int port_number;
  
    /**
     * Create a multiplexed color sensor.
     * 
     * @param i2cPort - What port the multiplexer is plugged into.
     * @param port_number    - What port the color sensor is plugged into the multiplexer
     *                <br>
     *                (commonly labeled SC3 and SD3 on the PCB, where 3 is the
     *                port)</br>
     */
    private MultiplexedColorSensor(I2C.Port i2cPort, int port_number) {
      if (i2c_multiplexer == null) {
        i2c_multiplexer = new I2C(i2cPort, HardwareConstants.I2C_MULTIPLEXER_ADDRESS);
      }
      this.port_number = port_number;
      setChannel();
      this.sensor = new ColorSensorV3(i2cPort);
    }

    /**
     * Since there is only one I2C port on roboRIO 2, so just pass in the port number on multiplexer should be enough
     * 
     * @param port_number which port the ColorSensorV3 is plugged on the multiplexer
     */
    public MultiplexedColorSensor(int port_number){
        this(I2C.Port.kOnboard, port_number);
    }
  
    /**
     * Helper method. This just sets the multiplexer to the correct port before
     * using the color sensor.
     */
    private void setChannel() {
      i2c_multiplexer.write(HardwareConstants.I2C_MULTIPLEXER_ADDRESS, 1 << this.port_number);
    }
  
    /*-----------------------------------------------------------------------*/
    /* Below are all of the methods used for the color sensor. */
    /* All this does is set the channel, then run the command on the sensor. */
    /*-----------------------------------------------------------------------*/
  
    public Color getColor() {
      setChannel();
      return this.sensor.getColor();
    }
  
    public int getProximity() {
      setChannel();
      return this.sensor.getProximity();
    }
  
    public RawColor getRawColor() {
      setChannel();
      return this.sensor.getRawColor();
    }
  
    public int getRed() {
      setChannel();
      return this.sensor.getRed();
    }
  
    public int getGreen() {
      setChannel();
      return this.sensor.getGreen();
    }
  
    public int getBlue() {
      setChannel();
      return this.sensor.getBlue();
    }
  
    public int getIR() {
      setChannel();
      return this.sensor.getIR();
    }
  
    public boolean hasReset() {
      setChannel();
      return this.sensor.hasReset();
    }
  }
