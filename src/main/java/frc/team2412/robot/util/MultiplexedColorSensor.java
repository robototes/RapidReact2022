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
    private I2C i2cMultiplexer;
    private final int i2cMultiplexerAddress;
    // The actual sensor. All of the methods call this sensor to get the data.
    private final ColorSensorV3 colorSensor;
    // What port on the multiplexer the color sensor is plugged into.
    private final int sensorPortNumber;
  
    /**
     * Create a multiplexed color sensor.
     * the constructor with most control, make it public if necessary
     */
    private MultiplexedColorSensor(I2C i2cMultiplexer, int i2cMultiplexerAddress, I2C.Port i2cPort, int sensorPortNumber) {
        this.i2cMultiplexer = i2cMultiplexer;
        this.i2cMultiplexerAddress = i2cMultiplexerAddress;
        this.sensorPortNumber = sensorPortNumber;
        setChannel();
        this.colorSensor = new ColorSensorV3(i2cPort);
    }

    /**
     * Since there is only one I2C port on roboRIO 2, so just pass in the port number on multiplexer should be enough
     * @param i2cMultiplexer the instance of I2C multiplexer 
     * @param sensorPortNumber which port the ColorSensorV3 is plugged on the multiplexer
     */
    public MultiplexedColorSensor(I2C i2cMultiplexer, int sensorPortNumber){
        this(i2cMultiplexer, HardwareConstants.I2C_MULTIPLEXER_ADDRESS, HardwareConstants.I2C_MULTIPLEXER_PORT, sensorPortNumber);
    }
  
    /**
     * Helper method. This just sets the multiplexer to the correct port before
     * using the color sensor.
     */
    private void setChannel() {
      this.i2cMultiplexer.write(this.i2cMultiplexerAddress, 1 << this.sensorPortNumber);
    }
  
    /*-----------------------------------------------------------------------*/
    /* Below are all of the methods used for the color sensor.               */
    /* All this does is set the channel, then run the command on the sensor. */
    /* Should covered most commonly used method, if not just add them below  */
    /*-----------------------------------------------------------------------*/
    public Color getColor() {
        setChannel();
        return this.colorSensor.getColor();
    }
  
    public int getProximity() {
        setChannel();
        return this.colorSensor.getProximity();
    }
  
    public RawColor getRawColor() {
        setChannel();
        return this.colorSensor.getRawColor();
    }
  
    public int getRed() {
        setChannel();
        return this.colorSensor.getRed();
    }
  
    public int getGreen() {
        setChannel();
        return this.colorSensor.getGreen();
    }
  
    public int getBlue() {
        setChannel();
        return this.colorSensor.getBlue();
    }
  
    public int getIR() {
        setChannel();
        return this.colorSensor.getIR();
    }
  
    public boolean hasReset() {
        setChannel();
        return this.colorSensor.hasReset();
    }
}
