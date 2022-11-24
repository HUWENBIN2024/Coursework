package comp3111.covid;


import java.io.IOException;
import java.net.URL;
import java.util.LinkedList;
import java.util.ResourceBundle;

import org.apache.commons.csv.CSVRecord;


import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.fxml.Initializable;
import javafx.scene.Node;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.CheckMenuItem;
import javafx.scene.control.DatePicker;
import javafx.scene.control.Menu;
import javafx.scene.control.Tab;
import javafx.scene.control.TextArea;
import javafx.scene.control.TextField;
import javafx.stage.Stage;
/**
 * Building on the sample skeleton for 'ui.fxml' Controller Class generated by SceneBuilder 
 */
public class Controller implements Initializable {
	
	private LinkedList <String> selectedcountryA = new LinkedList<String>();
	private LinkedList <String> selectedcountryB = new LinkedList<String>();
	private LinkedList <String> selectedcountryC = new LinkedList<String>();
	
    @FXML
    private Menu countrymenuA1;
    @FXML
    private Menu countrymenuB1;
    @FXML
    private Menu countrymenuC1;
	
    @FXML
    private DatePicker dateselectionbeginA2;
    @FXML
    private DatePicker dateselectionbeginB2;
    @FXML
    private DatePicker dateselectionbeginC2;
    
    @FXML
    private DatePicker dateselectionendA2;
    @FXML
    private DatePicker dateselectionendB2;
    @FXML
    private DatePicker dateselectionendC2;

    @FXML
    private Menu countrymenuA2;
    @FXML
    private Menu countrymenuB2;
    @FXML
    private Menu countrymenuC2;

    @FXML
    private Tab tabTaskZero;

    @FXML
    private TextField textfieldISO;

    @FXML
    private Button buttonConfirmedDeaths;

    @FXML
    private TextField textfieldDataset;

    @FXML
    private Button buttonRateOfVaccination;

    @FXML
    private Button buttonConfirmedCases;
    
    @FXML
    private DatePicker dateselectiontaskA;
    @FXML
    private DatePicker dateselectiontaskB;
    @FXML
    private DatePicker dateselectiontaskC;
    
    @FXML
    private Button EnterA1;
    @FXML
    private Button EnterB1;
    @FXML
    private Button EnterC1;
    
    @FXML
    private Button EnterA2;
    @FXML
    private Button EnterB2;
    @FXML
    private Button EnterC2;
    
    @FXML
    private Tab tabReport1;

    @FXML
    private Tab tabReport2;

    @FXML
    private Tab tabReport3;

    @FXML
    private Tab tabApp1;

    @FXML
    private Tab tabApp2;

    @FXML
    private Tab tabApp3;

    @FXML
    private TextArea textAreaConsole;

    
    /**
     *  Task Zero
     *  To be triggered by the "Confirmed Cases" button on the Task Zero Tab 
     *  
     */
    @FXML
    void doConfirmedCases(ActionEvent event) {
    	String iDataset = textfieldDataset.getText();
    	String iISO = textfieldISO.getText();
    	String oReport = DataAnalysis.getConfirmedCases(iDataset, iISO);
    	textAreaConsole.setText(oReport);
    }

  
    /**
     *  Task Zero
     *  To be triggered by the "Confirmed Deaths" button on the Task Zero Tab
     *  
     */
    @FXML
    void doConfirmedDeaths(ActionEvent event) {
    	String iDataset = textfieldDataset.getText();
    	String iISO = textfieldISO.getText();
    	String oReport = DataAnalysis.getConfirmedDeaths(iDataset, iISO);
    	textAreaConsole.setText(oReport);
    }

  
    /**
     *  Task Zero
     *  To be triggered by the "Rate of Vaccination" button on the Task Zero Tab
     *  
     */
    @FXML
    void doRateOfVaccination(ActionEvent event) {
    	String iDataset = textfieldDataset.getText();
    	String iISO = textfieldISO.getText();
    	String oReport = DataAnalysis.getRateOfVaccination(iDataset, iISO);
    	textAreaConsole.setText(oReport);
    }
    
    
    /**
     *  When this method is called, it will switch scene and pass the date to the TablecontrollerA
     */
    @FXML
    void switchscenetotableA1(ActionEvent event) throws IOException{
    	FXMLLoader tableloader = new FXMLLoader();
    	tableloader.setLocation(getClass().getResource("/tablegeneratorA.fxml"));
    	Parent tableviewA = tableloader.load();
    	Scene tableviewscene = new Scene(tableviewA);
    	// access the controller and call a method
    	TablecontrollerA tablecontroller = tableloader.getController();
    	try {
    		selectedcountryA.getFirst();
    		tablecontroller.initData(getdate(dateselectiontaskA),selectedcountryA);
        	Stage tablewindow = (Stage) ((Node)event.getSource()).getScene().getWindow(); 
        	tablewindow.setScene(tableviewscene);
        	tablewindow.show();
    	}
    	catch(Exception e) {
    		textAreaConsole.setText("please select at least one country and enter a specific date");
    	}
//    	
    }
    
    /**
     *  When this method is called, it will switch scene and pass the date to the TablecontrollerA
     */
    @FXML
    void switchscenetotableB1(ActionEvent event) throws IOException{
    	// we can go here

    	FXMLLoader tableloader = new FXMLLoader();

    	tableloader.setLocation(getClass().getResource("/tablegeneratorB.fxml"));
		
    	Parent tableviewB = tableloader.load();
  
    	Scene tableviewscene = new Scene(tableviewB);
 
    	// we can get here!
    	
    	// access the controller and call a method
    	TablecontrollerB tablecontroller = tableloader.getController();

    	
    	//we cant get here
    	try {
    		selectedcountryB.getFirst();
    		tablecontroller.initData(getdate(dateselectiontaskB),selectedcountryB);
        	Stage tablewindow = (Stage) ((Node)event.getSource()).getScene().getWindow(); 
        	tablewindow.setScene(tableviewscene);
        	tablewindow.show();
    	}
    	catch(Exception e) {
    		textAreaConsole.setText("please select at least one country and enter a specific date");
    	}
    }
    
    /**
     *  When this method is called, it will switch scene and pass the date to the TablecontrollerA
     */
    @FXML
    void switchscenetotableC1(ActionEvent event) throws IOException{
    	// we can go here

    	FXMLLoader tableloader = new FXMLLoader();

    	tableloader.setLocation(getClass().getResource("/tablegeneratorC.fxml"));
		
    	Parent tableviewC = tableloader.load();
  
    	Scene tableviewscene = new Scene(tableviewC);
 
    	// we can get here!
    	
    	// access the controller and call a method
    	TablecontrollerC tablecontroller = tableloader.getController();

    	
    	//we cant get here
    	try {
    		selectedcountryC.getFirst();
    		tablecontroller.initData(getdate(dateselectiontaskC),selectedcountryC);
        	Stage tablewindow = (Stage) ((Node)event.getSource()).getScene().getWindow(); 
        	tablewindow.setScene(tableviewscene);
        	tablewindow.show();
    	}
    	catch(Exception e) {
    		textAreaConsole.setText("please select at least one country and enter a specific date");
    	}
    }
    
    /**
     *  When this method is called, it will switch scene and pass the date to the ChartcontrollerA
     */
    @FXML
    void switchscenetochartA2(ActionEvent event) throws Exception {
    	FXMLLoader chartloader = new FXMLLoader();
    	chartloader.setLocation(getClass().getResource("/chartgeneratorA.fxml"));
    	Parent chartviewA = chartloader.load();
    	Scene chartviewscene = new Scene(chartviewA);
    	// access the controller and call a method
    	ChartcontrollerA chartcontroller = chartloader.getController();
    	try{
    		selectedcountryA.getFirst();		
    		if(comparedate(dateselectionbeginA2).compareTo(comparedate(dateselectionendA2))>=0) {
        		throw new Exception();
        	}       	
    		chartcontroller.initData(getdate(dateselectionbeginA2), getdate(dateselectionendA2), selectedcountryA);
        	Stage chartwindow = (Stage) ((Node)event.getSource()).getScene().getWindow(); 
    		chartwindow.setScene(chartviewscene);
    		chartwindow.show();
    	}
    	catch(Exception e) {
    		textAreaConsole.setText("please select at least one country and enter the specific date for begin and return"
    				+ "\nor you may check that the beginning date should be in front of end date.");
    	}
    }
    
    /**
     *  When this method is called, it will switch scene and pass the date to the ChartcontrollerA
     */
    @FXML
    void switchscenetochartB2(ActionEvent event) throws Exception {
    	FXMLLoader chartloader = new FXMLLoader();
    	chartloader.setLocation(getClass().getResource("/chartgeneratorB.fxml"));
    	Parent chartviewB = chartloader.load();
    	Scene chartviewscene = new Scene(chartviewB);
    	// access the controller and call a method
    	ChartcontrollerB chartcontroller = chartloader.getController();
    	try{
    		selectedcountryB.getFirst();		
    		if(comparedate(dateselectionbeginB2).compareTo(comparedate(dateselectionendB2))>=0) {
        		throw new Exception();
        	}       	
    		chartcontroller.initData(getdate(dateselectionbeginB2), getdate(dateselectionendB2), selectedcountryB);
        	Stage chartwindow = (Stage) ((Node)event.getSource()).getScene().getWindow(); 
    		chartwindow.setScene(chartviewscene);
    		chartwindow.show();
    	}
    	catch(Exception e) {
    		textAreaConsole.setText("please select at least one country and enter the specific date for begin and return"
    				+ "\nor you may check that the beginning date should be in front of end date.");
    	}
    }
    
    /**
     *  When this method is called, it will switch scene and pass the date to the ChartcontrollerA
     */
    @FXML
    void switchscenetochartC2(ActionEvent event) throws Exception {
    	FXMLLoader chartloader = new FXMLLoader();
    	chartloader.setLocation(getClass().getResource("/chartgeneratorC.fxml"));
    	Parent chartviewC = chartloader.load();
    	Scene chartviewscene = new Scene(chartviewC);
    	// access the controller and call a method
    	ChartcontrollerC chartcontroller = chartloader.getController();
    	try{
    		selectedcountryC.getFirst();		
    		if(comparedate(dateselectionbeginC2).compareTo(comparedate(dateselectionendC2))>=0) {
        		throw new Exception();
        	}       	
    		chartcontroller.initData(getdate(dateselectionbeginC2), getdate(dateselectionendC2), selectedcountryC);
        	Stage chartwindow = (Stage) ((Node)event.getSource()).getScene().getWindow(); 
    		chartwindow.setScene(chartviewscene);
    		chartwindow.show();
    	}
    	catch(Exception e) {
    		textAreaConsole.setText("please select at least one country and enter the specific date for begin and return"
    				+ "\nor you may check that the beginning date should be in front of end date.");
    	}
    }
    
   /**
    * transform the date into the format of dataset: MM/DD/YYYY (i.e. 1/3/2021)
    */
    String getdate(DatePicker dateselection) {
    	String date = dateselection.getValue().toString();
    	String year = date.substring(0,4);
	    String month = date.substring(5,7);
	    if (month.charAt(0)=='0')  month = month.substring(1);  // i.e. 01->1
	    String day = date.substring(8);
	    if (day.charAt(0)=='0')  day = day.substring(1);  // i.e. 01->1
	    String selecteddate = month+"/"+day+"/"+year; 
	    return selecteddate;
    }
    
    /**
     * compare the date between two DatePicker objects
     */
    String comparedate(DatePicker dateselection) {
    	return dateselection.getValue().toString();
    }

    /**
     * initialize the country menu(add all countries which can be selected into the menu)
     */
	@Override
	public void initialize(URL location, ResourceBundle resources) {
		EventHandler <ActionEvent> event = new EventHandler<ActionEvent>() {
			@Override
			public void handle(ActionEvent e) {
				if (((CheckMenuItem)e.getSource()).isSelected()) {
					selectedcountryA.add(((CheckMenuItem)e.getSource()).getText());
					selectedcountryB.add(((CheckMenuItem)e.getSource()).getText());
					selectedcountryC.add(((CheckMenuItem)e.getSource()).getText());
				}
				else {
					for(int i=0;i<selectedcountryA.size();i++) {
						if(((CheckMenuItem)e.getSource()).getText().equals(selectedcountryA.get(i))) {
							selectedcountryA.remove(i);
							selectedcountryB.remove(i);
							selectedcountryC.remove(i);
						}
					}
				}
			}			
		};
		String previous_country = "";
		String current_country = "";
		for(CSVRecord rec: DataAnalysis.getFileParser("COVID_Dataset_v1.0.csv")) {
			current_country = rec.get("location");
			if (!current_country.equals(previous_country)){
				
				// for taskA:
				CheckMenuItem item1 = new CheckMenuItem(current_country);
				CheckMenuItem item2 = new CheckMenuItem(current_country);
				item1.setOnAction(event);
				item2.setOnAction(event);
				countrymenuA1.getItems().add(item1);
				countrymenuA2.getItems().add(item2);
				
				// for taskB:
				CheckMenuItem item3 = new CheckMenuItem(current_country);
				CheckMenuItem item4 = new CheckMenuItem(current_country);
				item3.setOnAction(event);
				item4.setOnAction(event);
				countrymenuB1.getItems().add(item3);
				countrymenuB2.getItems().add(item4);
				
				// for taskC
				CheckMenuItem item5 = new CheckMenuItem(current_country);
				CheckMenuItem item6 = new CheckMenuItem(current_country);
				item5.setOnAction(event);
				item6.setOnAction(event);
				countrymenuC1.getItems().add(item5);
				countrymenuC2.getItems().add(item6);
				previous_country = current_country;
			}
		}
	}
	
	/**
	 * This function is just used for testing whether getdate() can get the proper date.
	 */
	public DatePicker getdatepickerA() {
		return dateselectionbeginA2;
	}
	/**
	 * This function is just used for testing whether getdate() can get the proper date.
	 */
	public DatePicker getdatepickerB() {
		return dateselectionbeginB2;
	}
	/**
	 * This function is just used for testing whether getdate() can get the proper date.
	 */
	public DatePicker getdatepickerC() {
		return dateselectionbeginC2;
	}
}

