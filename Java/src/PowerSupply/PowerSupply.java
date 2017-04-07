package PowerSupply;

import java.io.FileNotFoundException;

import javax.swing.JFrame;

public class PowerSupply {

	final static int SCREEN_WIDTH = 350, SCREEN_HEIGHT = 300;
	private JFrame window;
	
	public static void main(String[] args) throws FileNotFoundException {
		new PowerSupply();

	}
	public PowerSupply() throws FileNotFoundException{
		window = new JFrame("SmartSupply");
		window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		window.setSize(SCREEN_WIDTH, SCREEN_HEIGHT);

		settingPanel mainPanel = new settingPanel(this);
		window.add(mainPanel);

		window.setResizable(false);
		window.setVisible(true);
	}
}
