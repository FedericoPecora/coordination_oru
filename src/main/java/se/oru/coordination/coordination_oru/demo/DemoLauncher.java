package se.oru.coordination.coordination_oru.demo;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.HashMap;
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

import se.oru.coordination.coordination_oru.util.StringUtils;

public class DemoLauncher {

	private static final String testsPackage = "se.oru.coordination.coordination_oru.tests";
	
	private static void printUsage() {

		System.out.println("Usage: ./gradlew run -Pdemo=<demo>\n\nAvailable options for <demo>");

		List<ClassLoader> classLoadersList = new LinkedList<ClassLoader>();
		classLoadersList.add(ClasspathHelper.contextClassLoader());
		classLoadersList.add(ClasspathHelper.staticClassLoader());

		Reflections reflections = new Reflections(new ConfigurationBuilder()
		    .setScanners(new SubTypesScanner(false /* don't exclude Object.class */), new ResourcesScanner())
		    .setUrls(ClasspathHelper.forClassLoader(classLoadersList.toArray(new ClassLoader[0])))
		    .filterInputsBy(new FilterBuilder().include(FilterBuilder.prefix(testsPackage))));
		
		Set<Class<?>> classes = reflections.getSubTypesOf(Object.class);
		TreeSet<String> sortedClasses = new TreeSet<String>();
		HashMap<String,String> classDescriptions = new HashMap<String, String>();
		for (Class<? extends Object> cl : classes) {
			if (!cl.getSimpleName().equals("")) {
				sortedClasses.add(cl.getName().replace(testsPackage+".",""));
				String descString = "";
				if (cl.getAnnotation(DemoDescription.class) != null) descString = cl.getAnnotation(DemoDescription.class).desc();
				classDescriptions.put(cl.getName().replace(testsPackage+".",""),descString);
			}
		}
		for (String className : sortedClasses) {
			System.out.println();				
			List<String> descStrings = StringUtils.description("   \u001B[1m\u001b[32m"+className+"\u001b[0m: ", classDescriptions.get(className), 72, 6);
			for (String ds : descStrings) System.out.println(ds);
		}
		
		System.out.println();
		String note = "Most examples require the ReedsSheppCarPlanner motion planner, which is provided via a"
				+ "linked library that has to be built and depends on ompl (http://ompl.kavrakilab.org/)"
				+ "and mrpt (http://www.mrpt.org/). See REAME.md for building instructions.";
		List<String> formattedNote = StringUtils.description("NOTE: ", note, 72);
		for (String line : formattedNote) System.out.println(line);
		
	}
	
	public static void main(String[] args) throws ClassNotFoundException {
		
		//Forces to loads the class so that license and (c) are printed even if no demo is invoked
		Class.forName("se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator");
		
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
