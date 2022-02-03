package frc.team2412.robot.util;

import static frc.team2412.robot.Hardware.HardwareConstants.I2C_MULTIPLEXER_ADDRESS;
import static frc.team2412.robot.Hardware.HardwareConstants.I2C_MULTIPLEXER_PORT;

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
    // Constants from Hardware.java
    private static final I2C i2cMultiplexer = new I2C(I2C_MULTIPLEXER_PORT, I2C_MULTIPLEXER_ADDRESS);
    private static final int i2cMultiplexerAddress = I2C_MULTIPLEXER_ADDRESS;
    // The actual sensor. All of the methods call this sensor to get the data.
    private final ColorSensorV3 colorSensor;
    // What port on the multiplexer the color sensor is plugged into.
    private final int sensorPortNumber;

    private static int lastSensorPortNumber = -1;

    /**
     * Create a multiplexed color sensor.
     * the constructor with most control, make it public if necessary
     * assuming when initialize these subsystems is in sequential order so may not need a lock at here
     */
    private MultiplexedColorSensor(I2C.Port i2cPort, int sensorPortNumber) {
        this.sensorPortNumber = sensorPortNumber;
        setChannel();
        this.colorSensor = new ColorSensorV3(i2cPort);
    }

    /**
     * Since there is only one I2C port on roboRIO 2, so just pass in the port number on multiplexer should be enough
     * @param sensorPortNumber which port the ColorSensorV3 is plugged on the multiplexer
     */
    public MultiplexedColorSensor(int sensorPortNumber){
        this(HardwareConstants.I2C_MULTIPLEXER_PORT, sensorPortNumber);
    }

    /**
     * Helper method. This just sets the multiplexer to the correct port before using the color sensor.
     * checking if is still on the same port as last time, if so will skip this step
     */
    private void setChannel() {
        if (lastSensorPortNumber != this.sensorPortNumber) {
            i2cMultiplexer.write(i2cMultiplexerAddress, 1 << this.sensorPortNumber);
            lastSensorPortNumber = this.sensorPortNumber;
        }
    }
  
    /*-----------------------------------------------------------------------*/
    /* Below are all of the methods used for the color sensor.               */
    /* All this does is set the channel, then run the command on the sensor. */
    /* Should covered most commonly used method, if not just add them below  */
    /* Basically the synchronized (i2cMultiplexer) is putting a lock on the  */
    /* i2cMultiplexer, will release it after finish these code in the block  */
    /*-----------------------------------------------------------------------*/
    public Color getColor() {
        synchronized (i2cMultiplexer){
            setChannel();
            return this.colorSensor.getColor();
        }
    }
  
    public int getProximity() {
        synchronized (i2cMultiplexer){
            setChannel();
            return this.colorSensor.getProximity();
        }
    }
  
    public synchronized RawColor getRawColor() {
        synchronized (i2cMultiplexer){
            setChannel();
            return this.colorSensor.getRawColor();
        }
    }
  
    public synchronized int getRed() {
        synchronized (i2cMultiplexer){
            setChannel();
            return this.colorSensor.getRed();
        }
    }
  
    public synchronized int getGreen() {
        synchronized (i2cMultiplexer){
            setChannel();
            return this.colorSensor.getGreen();
        }
    }
  
    public synchronized int getBlue() {
        synchronized (i2cMultiplexer){
            setChannel();
            return this.colorSensor.getBlue();
        }
    }
  
    public synchronized int getIR() {
        synchronized (i2cMultiplexer){
            setChannel();
            return this.colorSensor.getIR();
        }
    }
  
    public boolean hasReset() {
        synchronized (i2cMultiplexer){
            setChannel();
            return this.colorSensor.hasReset();
        }
    }
}
