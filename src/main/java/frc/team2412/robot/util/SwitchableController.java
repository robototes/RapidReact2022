package frc.team2412.robot.util;

import kotlin.collections.SetsKt;
import org.frcteam2910.common.robot.input.Controller;
import org.frcteam2910.common.robot.input.XboxController;
import org.jetbrains.annotations.Nullable;

import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

public class SwitchableController<T, U extends Controller> {
    private final Map<T, OptionalController<U>> controllers;
    private T choice;
    public SwitchableController(Map<T, U> map){
        this(map, null);
    }
    public SwitchableController(Map<T, U> map, T start){
        choice = start;
        controllers = new LinkedHashMap<>();
        map.forEach((a, b) -> controllers.put(a, new OptionalController<>(b)));
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


    static{
        Controller a, b, c;

        SwitchableController<Integer, XboxController> s = new SwitchableController<>(Map.of(
                0, new XboxController(0),
                1, new XboxController(1),
                2, new XboxController(2)));

        a=s.getController(0);
        b=s.getController(1);
        c=s.getController(2);

        s.activate(0);


    }

}
