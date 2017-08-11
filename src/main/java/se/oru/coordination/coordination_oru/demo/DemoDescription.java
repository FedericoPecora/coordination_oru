package se.oru.coordination.coordination_oru.demo;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Retention(RetentionPolicy.RUNTIME)

public @interface DemoDescription {
	
	public String desc() default "N/A";
	
}
