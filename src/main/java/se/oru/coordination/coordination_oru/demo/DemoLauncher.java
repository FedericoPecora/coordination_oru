package se.oru.coordination.coordination_oru.demo;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

import org.reflections.Reflections;
import org.reflections.scanners.ResourcesScanner;
import org.reflections.scanners.SubTypesScanner;
import org.reflections.util.ClasspathHelper;
import org.reflections.util.ConfigurationBuilder;
import org.reflections.util.FilterBuilder;

public class DemoLauncher {

	private static final String testsPackage = "se.oru.coordination.coordination_oru.tests";

	private static void printUsage() {
		
		String st = "\nUsage: ./gradlew run -Pdemo=<demo>\n\nAvailable options for <demo>";
		
		List<ClassLoader> classLoadersList = new LinkedList<ClassLoader>();
		classLoadersList.add(ClasspathHelper.contextClassLoader());
		classLoadersList.add(ClasspathHelper.staticClassLoader());

		Reflections reflections = new Reflections(new ConfigurationBuilder()
		    .setScanners(new SubTypesScanner(false /* don't exclude Object.class */), new ResourcesScanner())
		    .setUrls(ClasspathHelper.forClassLoader(classLoadersList.toArray(new ClassLoader[0])))
		    .filterInputsBy(new FilterBuilder().include(FilterBuilder.prefix(testsPackage))));
		
		Set<Class<?>> classes = reflections.getSubTypesOf(Object.class);
		int length = 0;
		for (Class<? extends Object> cl : classes) {
			if (!cl.getSimpleName().equals("")) length = Math.max(length, cl.getSimpleName().length());
		}
		TreeSet<String> orderedClasses = new TreeSet<String>();
		
		for (Class<? extends Object> cl : classes) {
			DemoDescription desc = cl.getAnnotation(DemoDescription.class);
			if (!cl.getSimpleName().equals("")) {
				int buffer = length-cl.getSimpleName().length()+5;
				String bufferString = " ";
				for (int i = 0; i < buffer; i++) bufferString += ".";
				bufferString += " ";
				orderedClasses.add(cl.getSimpleName() + (desc != null ? bufferString + desc.desc() : ""));
			}
		}			
		for (String cln : orderedClasses) {
			st += "\n    " +cln;
		}
		System.out.println(st);
		System.out.println("\nNOTE: Most examples require the ReedsSheppCarPlanner motion planner, which is provided via a"
				         + "\n      linked library that has to be built and depends on ompl (http://ompl.kavrakilab.org/)"
				         + "\n      and mrpt (http://www.mrpt.org/) - see REAME.md for building instructions.");
	}
	
	public static void main(String[] args) {
		if (args.length != 1) printUsage();
		else {
			String className = args[0];
			try {
				Class<?> cl = Class.forName(testsPackage+"."+className);
				Method meth = cl.getMethod("main", String[].class);
			    String[] params = null;
			    meth.invoke(null, (Object) params);
			}
			catch (IllegalAccessException e) { e.printStackTrace(); }
			catch (ClassNotFoundException e) { e.printStackTrace(); }
			catch (NoSuchMethodException e) { e.printStackTrace(); }
			catch (SecurityException e) { e.printStackTrace(); }
			catch (IllegalArgumentException e) { e.printStackTrace(); }
			catch (InvocationTargetException e) { e.printStackTrace(); }
		}
	}

}
