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

import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.util.StringUtils;

public class DemoLauncher {

	private static final String testsPackage = "se.oru.coordination.coordination_oru.tests";

	private static void printUsage() {

		System.out.println("\n"+TrajectoryEnvelopeCoordinator.TITLE);
		System.out.println(TrajectoryEnvelopeCoordinator.COPYRIGHT+"\n");
		if (TrajectoryEnvelopeCoordinator.LICENSEE != null) {
			List<String> lic = StringUtils.fitWidth(TrajectoryEnvelopeCoordinator.PRIVATE_LICENSE, 72, 5);
			for (String st : lic) System.out.println(st);
		}
		else {
			List<String> lic = StringUtils.fitWidth(TrajectoryEnvelopeCoordinator.PUBLIC_LICENSE, 72, 5);
			for (String st : lic) System.out.println(st);
		}

		System.out.println("\nUsage: ./gradlew run -Pdemo=<demo>\n\nAvailable options for <demo>");

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
		
//		String CONNECTOR_LEAF = (char)0x2514 + "" + (char)0x2500 + " ";

		for (Class<? extends Object> cl : classes) {
			DemoDescription desc = cl.getAnnotation(DemoDescription.class);
			if (!cl.getSimpleName().equals("")) {
				System.out.println();				
				String descString = "";
				if (desc != null) {
					descString = desc.desc();
				}
				List<String> descStrings = StringUtils.description("   "+cl.getSimpleName()+": ", descString, 72, 6);
				for (String ds : descStrings) System.out.println(ds);
//				System.out.println("\n   "+cl.getSimpleName());
//				if (desc != null) {
//					List<String> descStrings = StringUtils.description("   "+CONNECTOR_LEAF,desc.desc(), 72);
//					for (String ds : descStrings) System.out.println(ds);
//				}
			}
		}			
		
		System.out.println();
		String note = "Most examples require the ReedsSheppCarPlanner motion planner, which is provided via a"
				+ "linked library that has to be built and depends on ompl (http://ompl.kavrakilab.org/)"
				+ "and mrpt (http://www.mrpt.org/). See REAME.md for building instructions.";
		List<String> formattedNote = StringUtils.description("NOTE: ", note, 72);
		for (String line : formattedNote) System.out.println(line);
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
