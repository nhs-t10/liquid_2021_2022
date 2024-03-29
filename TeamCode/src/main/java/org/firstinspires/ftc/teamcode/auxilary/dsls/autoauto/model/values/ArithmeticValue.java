package org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.values;

import com.google.firebase.database.annotations.NotNull;

import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.Location;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.AutoautoRuntimeVariableScope;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.errors.AutoautoNameException;

public class ArithmeticValue extends AutoautoValue {
    String operator;
    AutoautoValue right;
    AutoautoValue left;

    AutoautoPrimitive returnValue;

    private AutoautoRuntimeVariableScope scope;
    private Location location;

    public static ArithmeticValue O(AutoautoValue left, String operator, AutoautoValue right) {
        return new ArithmeticValue(left, operator, right);
    }

    public ArithmeticValue(AutoautoValue left, String operator, AutoautoValue right) {
        this.left = left;
        this.operator = operator;
        this.right = right;
    }
    @Override
    public void init() {
        left.init();
        right.init();
    }

    public void loop() {
        left.loop();
        right.loop();

        AutoautoValue leftRes = left.getResolvedValue();
        AutoautoValue rightRes = right.getResolvedValue();

        if(leftRes instanceof AutoautoString || rightRes instanceof AutoautoString) {
            if(operator.equals("+")) concatenate(left.asString(), right.asString());
            else throw new AutoautoNameException("[AUTOAUTO ERROR] Bad operator " + operator + "on string value.");

            return;
        }

        float a = ((AutoautoNumericValue)leftRes).getFloat();
        float b = ((AutoautoNumericValue)rightRes).getFloat();

        switch(operator) {
            case "%":
                this.returnValue = new AutoautoNumericValue(a % b);
                break;
            case "^":
                this.returnValue = new AutoautoNumericValue((float) Math.pow(a, b));
                break;
            case "*":
                this.returnValue = new AutoautoNumericValue(a * b);
                break;
            case "/":
                this.returnValue = new AutoautoNumericValue(a / b);
                break;
            case "+":
                this.returnValue = new AutoautoNumericValue(a + b);
                break;
            case "-":
                this.returnValue = new AutoautoNumericValue(a - b);
                break;
            default:
                this.returnValue = new AutoautoNumericValue(a);
                break;
        }
    }

    @Override
    public String getString() {
        return returnValue.getString();
    }

    @Override
    public ArithmeticValue clone() {
        ArithmeticValue c = new ArithmeticValue(left.clone(), operator, right.clone());
        c.setLocation(location);
        return c;
    }

    private void concatenate(AutoautoString a, AutoautoString b) {
        this.returnValue = new AutoautoString(a.getString() + b.getString());
    }

    @NotNull
    public String toString() {
        return left + " " + operator + " " + right;
    }

    @NotNull
    @Override
    public AutoautoPrimitive getResolvedValue() {
        return this.returnValue;
    }

    @Override
    public AutoautoRuntimeVariableScope getScope() {
        return scope;
    }

    @Override
    public void setScope(AutoautoRuntimeVariableScope scope) {
        this.scope = scope;
        left.setScope(scope);
        right.setScope(scope);
    }

    @Override
    public Location getLocation() {
        return location;
    }

    @Override
    public void setLocation(Location location) {
        this.location = location;
    }
}
