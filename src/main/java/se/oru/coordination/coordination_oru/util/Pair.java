package se.oru.coordination.coordination_oru.util;

public class Pair<T extends Comparable<T>> {
	
    private T p1, p2;
    
    public Pair(T p1, T p2) {
        this.p1 = p1;
        this.p2 = p2;
    }
    
    public T getFirst() {
    	return p1;
    }

    public T getSecond() {
    	return p2;
    }

    public boolean contains(Pair<T> otherPair) {
    	return 	(this.p1.compareTo(otherPair.p1) < 0 && this.p2.compareTo(otherPair.p2) > 0) ||
    			(this.p1.compareTo(otherPair.p1) < 0 && this.p2.compareTo(otherPair.p2) >= 0) ||
    			(this.p1.compareTo(otherPair.p1) <= 0 && this.p2.compareTo(otherPair.p2) > 0);
    }
    
    public boolean containsOrEquals(Pair<T> otherPair) {
    	return this.p1.compareTo(otherPair.p1) <= 0 && this.p2.compareTo(otherPair.p2) >= 0;
    }
    
    @Override
    public int hashCode() {
    	return this.toString().hashCode();
    }
    
    @Override
    public boolean equals(Object otherPair){
        if (otherPair instanceof Pair<?> && ((Pair<?>)otherPair).p1.equals(this.p1) && ((Pair<?>)otherPair).p2.equals(this.p2)) return true;
        return false;
    }
    
    @Override
    public String toString() {
    	return "(" + this.p1 + "," + this.p2 + ")";
    }
    
}