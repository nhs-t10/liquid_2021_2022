package org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.values;

import com.google.firebase.database.annotations.NotNull;

public abstract class AutoautoPrimitive extends AutoautoValue {
    @NotNull
    @Override
    public AutoautoPrimitive getResolvedValue() {
        return this;
    }
    public abstract String getJSONString();
}
