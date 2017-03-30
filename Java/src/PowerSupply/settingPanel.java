package PowerSupply;

import java.awt.Font;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.JTextField;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;

import com.fazecast.jSerialComm.*;

public class settingPanel extends JPanel implements ActionListener  {
	// Initialise serial port
	public SerialPort port;
	// variables
	private JLabel labelList[] = new JLabel[4];
	private String[] labelNames = {"Set Voltage (V)","Set Current (mA)","Output Voltage","Output Current"};
	private JTextField[] textFieldList = new JTextField[2];
	private JTextField[] emptyTextFieldList = new JTextField[2];
	private JTextField[] magnitudeTextFieldList = new JTextField[2];
	private String[] buttonNames = {"PWR", "Set", "Set"};
	private JButton[] buttonList = new JButton[3];
	
	public settingPanel(PowerSupply window) throws FileNotFoundException{
		this.setLayout(null);
		//drop down menu
		JComboBox<String> portList = new JComboBox<String>(); 
		portList.setBounds(25,25,100,25);
		this.add(portList);
		//connect button
		JButton connectButton = new JButton("Connect");		  
		connectButton.setBounds(150, 25, 100, 25);
		this.add(connectButton);
		
		//buttons & textfields
		for (int i=0;i<labelList.length;i++)
		{
			labelList[i]=new JLabel("");
			labelList[i].setBounds(25,75+50*i,100,25); 
			labelList[i].setText(labelNames[i]);
			this.add(labelList[i]);
		}
		for (int i=0;i<textFieldList.length;i++)
		{
			textFieldList[i]=new JTextField("");
			textFieldList[i].setBounds(150,75+50*i,100,25); 
			textFieldList[i].addActionListener(this);
			this.add(textFieldList[i]);
		}
		for (int i=0;i<emptyTextFieldList.length;i++)
		{
			emptyTextFieldList[i]=new JTextField("");
			emptyTextFieldList[i].setBounds(150,175+50*i,100,25); 
			emptyTextFieldList[i].setFocusable(false);
			this.add(emptyTextFieldList[i]);
		}
		for (int i=0;i<magnitudeTextFieldList.length;i++)
		{
			magnitudeTextFieldList[i]=new JTextField("");
			magnitudeTextFieldList[i].setBounds(275,175+50*i,60,25); 
			magnitudeTextFieldList[i].setFocusable(false);
			this.add(magnitudeTextFieldList[i]);
		}
		magnitudeTextFieldList[0].setText("V");
		magnitudeTextFieldList[1].setText("mA");
		for (int i=0;i<buttonList.length;i++)
		{

			buttonList[i]=new JButton(buttonNames[i]);
			buttonList[i].setFont(new Font("Arial", Font.BOLD, 10));
			buttonList[i].addActionListener(this);
			buttonList[i].setBounds(275,25+50*i,60,25); 
			buttonList[i].setEnabled(false);
			this.add(buttonList[i]);
		}		
		
		//new file to save sensordata
		PrintWriter sensordata = new PrintWriter(new File("Sensordata.csv"));
		sensordata.write("Voltage(V)");
		sensordata.write(',');
		sensordata.write("Current(mA)");
		sensordata.write('\n');

			
		//list available serial ports
		SerialPort[] portNames = SerialPort.getCommPorts();		
		for(int i = 0; i < portNames.length; i++)
			portList.addItem(portNames[i].getSystemPortName());
		
		
		//actionlistener for connect button (needs to stay inside settingPanel)
		connectButton.addActionListener(new ActionListener(){
			@Override public void actionPerformed(ActionEvent arg0) {
				if(connectButton.getText().equals("Connect")) {
					// try to connect to serial port
					port = SerialPort.getCommPort(portList.getSelectedItem().toString());
					port.setComPortTimeouts(SerialPort.TIMEOUT_SCANNER, 0, 0);
					if(port.openPort()) {
						connectButton.setText("Disconnect");
						portList.setEnabled(false);
						// activate buttons
						for (int i=0;i<buttonList.length;i++)
						{
							buttonList[i].setEnabled(true);
						}
						
						port.addDataListener(new SerialPortDataListener() {
							@Override
							public int getListeningEvents() { return SerialPort.LISTENING_EVENT_DATA_AVAILABLE; }
							// make stringbuilder to concatenate chars
							StringBuilder sb = new StringBuilder();
							boolean voltageOrCurrent = false;
							boolean limiting = false;
							@Override
							public void serialEvent(SerialPortEvent event) {
								if (event.getEventType() != SerialPort.LISTENING_EVENT_DATA_AVAILABLE)
									return;
								// read available data into array (port.bytesAvailable gives amount of available bytes) 
								byte[] newData = new byte[port.bytesAvailable()];
								port.readBytes(newData, newData.length);
								// convert each element from ascii to number
								// '>' ends the string (14)
								// '.' is also a separate case (-2)
								// furthermore 'L' indicates current limiting (28)
								for (int i = 0; i < newData.length; i++){
									int data = newData[i] - 48;				//convert from ascii to int
									if (data == -2){
										sb.append(".");
									}
									else if (data == 28){
										limiting = true;
									}
									else if (data != 14){
										sb.append(data);
									}
									else{
										sensordata.write(sb.toString());
										sensordata.write(',');
										//voltage
										if (voltageOrCurrent == false){ 
											emptyTextFieldList[0].setText(sb.toString());
										}
										//current
										else{
											emptyTextFieldList[1].setText(sb.toString());
											if (limiting == true){
												sensordata.write("Limiting");
												sensordata.write(',');
												limiting = false;
											}
											sensordata.write('\n');
										}
										sb.setLength(0);
										voltageOrCurrent = !voltageOrCurrent;
									}
								}
							}

						});		
					}
				} 
				else {
					// disconnect from serial port
					port.closePort();
					portList.setEnabled(true);
					connectButton.setText("Connect");
					// deactivate buttons
					for (int i=0;i<buttonList.length;i++)
					{
						buttonList[i].setEnabled(false);
					}
					// write to file & close it
					sensordata.close();
				}
			}
		});	
	}

	// actionevents for buttons & pressing enter inside textfield
	@Override
	public void actionPerformed(ActionEvent actionEvent) {
		PrintWriter output = new PrintWriter(port.getOutputStream());
		//ON/OFF: send 9001
		if (actionEvent.getSource() == buttonList[0]) 
		{
			output.print("9001");
			output.flush();
		} 
		//Voltage: send number 0-999
		if (actionEvent.getSource() == buttonList[1] || actionEvent.getSource() == textFieldList[0]) 
		{
			float voltage = 100*Float.parseFloat(textFieldList[0].getText());
			if (voltage < 1000 && voltage >= 0){
				output.print(voltage);
				output.flush();
			}
		}
		//Current: send number 1000-1999
		if (actionEvent.getSource() == buttonList[2] || actionEvent.getSource() == textFieldList[1]) 
		{
			int current = Integer.parseInt(textFieldList[1].getText());
			if (current < 1000 && current >= 0){
				output.print(1000+current);
				output.flush();
			}
		}		
	}
}
