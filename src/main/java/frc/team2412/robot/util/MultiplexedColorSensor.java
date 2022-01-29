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
    private I2C i2c_multiplexer;
    private final int i2c_multiplexer_address;
    // The actual sensor. All of the methods call this sensor to get the data.
    private final ColorSensorV3 sensor;
    // What port on the multiplexer the color sensor is plugged into.
    private final int sensor_port_number;
  
    /**
     * Create a multiplexed color sensor.
     * the constructor with most control, make it public if necessary
     */
    private MultiplexedColorSensor(I2C i2c_multiplexer, int i2c_multiplexer_address, I2C.Port i2cPort, int sensor_port_number) {
        this.i2c_multiplexer = i2c_multiplexer;
        this.i2c_multiplexer_address = i2c_multiplexer_address;
        this.sensor_port_number = sensor_port_number;
        setChannel();
        this.sensor = new ColorSensorV3(i2cPort);
    }

    /**
     * Since there is only one I2C port on roboRIO 2, so just pass in the port number on multiplexer should be enough
     * @param i2c_multiplexer the instance of I2C multiplexer 
     * @param sensor_port_number which port the ColorSensorV3 is plugged on the multiplexer
     */
    public MultiplexedColorSensor(I2C i2c_multiplexer, int sensor_port_number){
        this(i2c_multiplexer, HardwareConstants.I2C_MULTIPLEXER_ADDRESS, HardwareConstants.I2C_MULTIPLEXER_PORT, sensor_port_number);
    }
  
    /**
     * Helper method. This just sets the multiplexer to the correct port before
     * using the color sensor.
     */
    private void setChannel() {
      this.i2c_multiplexer.write(this.i2c_multiplexer_address, 1 << this.sensor_port_number);
    }
  
    /*-----------------------------------------------------------------------*/
    /* Below are all of the methods used for the color sensor.               */
    /* All this does is set the channel, then run the command on the sensor. */
    /* Should covered most commonly used method, if not just add them below  */
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
