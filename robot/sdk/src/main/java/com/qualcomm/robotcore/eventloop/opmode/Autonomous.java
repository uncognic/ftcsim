package com.qualcomm.robotcore.eventloop.opmode;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Documented
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
public @interface Autonomous {
    String name() default "";

    String group() default "";

    /** Name of the TeleOp to select on the Driver Station after this OpMode ends. */
    String preselectTeleOp() default "";
}
