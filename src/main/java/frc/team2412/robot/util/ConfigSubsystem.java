package frc.team2412.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.util.Arrays;

public abstract class ConfigSubsystem extends SubsystemBase implements Loggable {
    @Retention(RetentionPolicy.RUNTIME)
    @Target(value = { ElementType.TYPE })
    public @interface Constants {
        // any subsystems can use this constant
        Class<?> value() default ConfigSubsystem.class;
    }

    protected Loggable constantRoot = null;

    public ConfigSubsystem() {
        generateConstants(getClass());
    }

    public ConfigSubsystem(Class<?> c) {
        generateConstants(c);
    }

    public void generateConstants(Class<?> root) {
        Arrays.stream(root.getDeclaredClasses())
                .filter(c -> c.isAnnotationPresent(Constants.class)
                        && c.getDeclaredAnnotation(Constants.class).value().isAssignableFrom(root))
                .forEach(c -> {
                    try {
                        constantRoot = (Loggable) c.newInstance();
                    } catch (InstantiationException | IllegalAccessException e) {
                        System.out.println("you done goof");
                    }
                });
    }
}
