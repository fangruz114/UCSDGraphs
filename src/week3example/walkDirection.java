package week3example;


//You have a walking robot. Given a string of compass directions 
//(e.g., "nesw" or "nnssen"), it will travel 30 seconds in each 
//of those directions. Write a method to determine whether a set 
//of directions will lead your robot back to its starting position.

//"nnnn"  => false
//"nenessww"  => true


public class walkDirection {
	
	private static int x;
	private static int y;
	
	public walkDirection() {
		x=0;
		y=0;
	}
	
	public static boolean walkPath(String direct) {
		if (direct == null) {
			return true;
		}
		String direction = direct.toLowerCase();
		for (char d: direction.toCharArray()) {
			if (d == 'n') {
				y++;
			}
			if (d == 'e'){
				x++;
			}
			if (d == 's') {
				y--;
			}
			if (d == 'w') {
				x--;
			}
		}
		if (x == 0 && y == 0) {
			return true;
		}
		return false;
	}
    
	public static void main(String args[]) {
		System.out.println("moving direction \"nenessww\"");
		System.out.println(walkPath("nenessww"));
		
		System.out.println("moving direction \"nnnn\"");
		System.out.println(walkPath("nnnn"));
	}
}
