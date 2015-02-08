public class Obstacles {
	int x;
	int y;

	public Obstacles(int x, int y) {
		this.x = x;
		this.y =y;
	}
	@Override
	public String toString(){
		return Integer.toString(x)+","+Integer.toString(y);
	}

}
