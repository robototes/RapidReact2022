package frc.team2412.robot.util;

import org.frcteam2910.common.robot.input.Controller;
import org.frcteam2910.common.robot.input.XboxController;
import org.jetbrains.annotations.Nullable;

import java.util.*;



@SuppressWarnings("unused")
public class SwitchableController<T, U extends Controller> {
    private final Map<T, OptionalController<U>> controllers;
    private T choice;
    public SwitchableController(Map<T, U> map){
        this(null, map);
    }
    public SwitchableController(T start, Map<T, U> map){
        choice = start;
        controllers = new LinkedHashMap<>();
        map.forEach((a, b) -> controllers.put(a, new OptionalController<>(b)));
    }
    @SafeVarargs
    public SwitchableController(Map.Entry<T, U>... args){
        this(Map.ofEntries(args));
    }
    @SafeVarargs
    public SwitchableController(T start, Map.Entry<T, U>... args){
        this(start, Map.ofEntries(args));
    }

    public T getChoice(){
        return choice;
    }
    @Nullable
    public OptionalController<U> getController(T key){
        return controllers.get(key);
    }
    @Nullable
    public OptionalController<U> getActiveController(){
        return getController(choice);
    }

    public boolean activate(T newChoice){
        if(choice == newChoice) return false;
        if(getActiveController() != null) getActiveController().deactivate();
        choice = newChoice;
        if(getActiveController() != null) getActiveController().deactivate();
        return true;
    }

    public boolean deactivate(T newChoice){
        return newChoice == choice && activate(null);
    }

    public boolean deactivate(){
        return deactivate(choice);
    }


    public enum Controllers {
        PRIMARY, SECONDARY, TERTIARY, BACKUP, DRIVER, CODRIVER, DEBUG, TEST;

        public Map.Entry<Controllers, XboxController> on(int port){
            return SwitchableController.on(this, port);
        }
    }

    public static <T extends Enum<T>> Map.Entry<T, XboxController> on(T num, int port){
        return new AbstractMap.SimpleEntry<>(num, new XboxController(port));
    }


    public static SwitchableController<Integer, XboxController> of(int... args){
        Map<Integer, XboxController> controllerMap = new LinkedHashMap<>();
        for(int i : args){
            controllerMap.put(i, new XboxController(i));
        }
        return new SwitchableController<>(controllerMap);
    }

 
}
