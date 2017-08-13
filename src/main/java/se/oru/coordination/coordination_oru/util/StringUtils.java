package se.oru.coordination.coordination_oru.util;

import java.util.ArrayList;
import java.util.List;

public class StringUtils {

	
	public static List<String> description(String heading, String paragraph, int maxWidth) {
		return description(heading, paragraph, maxWidth, heading.length());
	}
	
	public static List<String> description(String heading, String paragraph, int maxWidth, int indent) {
		String[] words = paragraph.split(" ");
		List<String> ret = new ArrayList<String>();
		if(words == null || words.length == 0) return ret;
		String indentString = "";
		for (int i = 0; i < indent; i++) indentString += " "; 
		String st = heading;
		for (int i = 0; i < words.length; i++) {
			String word = words[i].trim();
			if (word != "") {
				if (st.length()+word.length() < maxWidth) {
					st += word + " ";
					if (i == words.length-1) {
						if (ret.isEmpty()) ret.add(st.substring(0, st.length()-1));
						else ret.add(indentString+st.substring(0, st.length()-1));
					}
				}
				else {
					if (ret.isEmpty()) ret.add(st.substring(0, st.length()-1));
					else ret.add(indentString+st.substring(0, st.length()-1));
					st = "";
					if (i == words.length-1) {
						if (ret.isEmpty()) ret.add(st+word);
						else ret.add(indentString+word);
					}
					else i--;
				}
			}
		}
		return ret;
	}
	
	public static List<String> fitWidth(String paragraph, int maxWidth, int indent) {
		String[] words = paragraph.split(" ");
		List<String> ret = new ArrayList<String>();
		if(words == null || words.length == 0) return ret;
		String indentString = "";
		for (int i = 0; i < indent; i++) indentString += " "; 
		String st = "";
		for (int i = 0; i < words.length; i++) {
			String word = words[i].trim();
			if (word != "") {
				if (st.length()+word.length() < maxWidth) {
					st += word + " ";
					if (i == words.length-1) ret.add(indentString+st.substring(0, st.length()-1));
				}
				else {
					ret.add(indentString+st.substring(0, st.length()-1));
					st = "";
					if (i == words.length-1) ret.add(indentString+word);
					else i--;
				}
			}
		}
		return ret;
	}

	public static List<String> fullJustify(String paragraph, int maxWidth, int indent) {
		String[] words = paragraph.split(" ");
		String indentString = "";
		for (int i = 0; i < indent; i++) indentString += " ";
		List<String> result = new ArrayList<String>();
		if(words == null || words.length == 0) return result;
		int count = 0;
		int last = 0;
		for(int i = 0; i < words.length; i++) {
			count = count + words[i].length();
			if(count+i-last > maxWidth) {
				int wordsLen = count-words[i].length();
				int spaceLen = maxWidth-wordsLen;
				int eachLen = 1;
				int extraLen = 0;
				if(i-last-1 > 0) {
					eachLen = spaceLen/(i-last-1);
					extraLen = spaceLen%(i-last-1);
				}
				StringBuilder sb = new StringBuilder();
				for(int k = last; k < i-1; k++) {
					sb.append(words[k]);
					int ce = 0;
					while(ce < eachLen) {
						sb.append(" ");
						ce++;
					}
					if(extraLen > 0) {
						sb.append(" ");
						extraLen--;
					}
				}
				sb.append(words[i-1]);//last words in the line
				//if only one word in this line, need to fill left with space
				while(sb.length() < maxWidth) {
					sb.append(" ");
				}
				result.add(indentString+sb.toString());
				last = i;
				count=words[i].length();
			}
		}
		StringBuilder sb = new StringBuilder();
		for(int i = last; i < words.length-1; i++) {
			count = count+words[i].length();
			sb.append(words[i]+" ");
		}	 
		sb.append(words[words.length-1]);
		while(sb.length() < maxWidth) {
			sb.append(" ");
		}
		result.add(indentString+sb.toString());
		return result;
	}

}
