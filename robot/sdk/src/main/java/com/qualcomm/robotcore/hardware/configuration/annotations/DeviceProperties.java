package com.qualcomm.robotcore.hardware.configuration.annotations;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/** How a user-defined device appears in the robot configuration; {@code xmlTag} is its type name there. */
@Documented
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
public @interface DeviceProperties {
    String name();

    String xmlTag();

    String description() default "";

    boolean builtIn() default false;
}
