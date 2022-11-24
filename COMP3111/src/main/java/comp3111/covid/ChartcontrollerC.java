package comp3111.covid;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Cursor;
import javafx.scene.Node;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.chart.CategoryAxis;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.XYChart.Series;
import javafx.scene.chart.XYChart.Data;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.TextArea;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;
import javafx.stage.Stage;

/**
 * This is the controller class building on "chartgeneratorC.fxml" for generating chartC
 * @author HU, Wenbin
 * 
 */
public class ChartcontrollerC {
	
    @FXML
    private Button Return;
    
    @FXML
    private TextArea Textconsolechart;
    
    @FXML
    private LineChart<String, Number> linechart;
    
    /**
     * it is called by main controller to pass the data to chart controller.
     * @param begin
     * @param end
     * @param selectedcountry
     * 
     */
	public void initData(String begin, String end,LinkedList<String>selectedcountry) {
		 setupchart(begin,end,selectedcountry);
	}
	
	/**
	 * This is the main function to perform chart set up.
	 * @param begin
	 * @param end
	 * @param selectedcountry
	 * 
	 */
	void setupchart(String begin, String end,LinkedList<String>selectedcountry) {
		linechart.setTitle("Cumulative Rate of Vaccination against COVID-19 Deaths");
		linechart.setCursor(Cursor.CROSSHAIR);
        TaskC2 temp = new TaskC2();
        ArrayList<String> datalist = temp.storedata(begin, end, selectedcountry);
        try {
        	String[] split_data = datalist.get(0).split(" ");
        	String previous_country = "hhh";
        	HashSet<String> available_countrylist = temp.getAvailable_country();
        	int number_of_country = available_countrylist.size();
        	int count =0;
			@SuppressWarnings("unchecked")
			Series<String,Number>[] series = new Series[number_of_country];
        	String chartreport ="";
        	String begin_date ="";  // this is used to record the beginning date of particular country
        	String end_date = "";  // this is used to record the end date of particular country
        	for(int i=0;i<datalist.size();i++) {
        		split_data = datalist.get(i).split(",");
        		String current_country = split_data[0];
        		String current_date = split_data[1];
        		String confirmedperm = split_data[2];
        		if(!current_country.equals(previous_country)) {
        			series[count]= new Series<String, Number>();
        			linechart.getData().add(series[count]);
        			series[count].setName(current_country);
        			begin_date = current_date;
        			if (!begin_date.equals(TaskC2.transformdate(begin))) {
        				String tempreport = "We just have data of " + current_country+" from " + begin_date;
        				chartreport += tempreport;
        				if(i == datalist.size()-1 || !datalist.get(i+1).split(",")[0].equals(current_country)) {
        					chartreport = chartreport+ " to "+current_date+"(the data we have begins at "+begin_date+")"+"\n";
        				}
        			}
        			// i is the last data or the next data country is not equal to current country(the last date)
    				// this is the situation we only have one data points of this country
        			else if(i == datalist.size()-1 || !datalist.get(i+1).split(",")[0].equals(current_country)) {
    					chartreport = "We just have data of " + current_country+" from " + begin_date+" to "+current_date+"(the data we have ends at "+current_date+")"+"\n";
    				}
        			Data <String, Number> nodedata = new Data<String, Number>(current_date, Float.parseFloat(confirmedperm));
        			series[count].getData().add(nodedata);
        			String countryname = series[count].getName();
        			nodedata.setNode(new HoveredNode(countryname,nodedata.getXValue().toString(),nodedata.getYValue().floatValue()));
        			previous_country = current_country;
        			count +=1;
        		}
        		// where the data is not the beginning date(middle point or end point)
        		else {
        			end_date = current_date;
        			Data <String, Number> nodedata = new Data<String, Number>(current_date, Float.parseFloat(confirmedperm));
        			String countryname = series[count-1].getName();
        			nodedata.setNode(new HoveredNode(countryname,nodedata.getXValue().toString(),nodedata.getYValue().floatValue()));
        			series[count-1].getData().add(nodedata);
        			if(begin_date.equals(TaskC2.transformdate(begin)) && (i == datalist.size()-1 || (!datalist.get(i+1).split(",")[0].equals(current_country)))
        					&& (!end_date.equals(TaskC2.transformdate(end)))) {
        				String tempreport = "We just have data of " +current_country+" from " + begin_date+" to "+end_date+"(the data we have ends at "+end_date+")"+"\n";
        				chartreport +=tempreport;
        			}
        			// when the beginning date is not equal to the begin date we require.
        			else if((!begin_date.equals(TaskC2.transformdate(begin))) &&(i == datalist.size()-1 || (!datalist.get(i+1).split(",")[0].equals(current_country)))) {
        				if (!end_date.equals(TaskC2.transformdate(end))) {
        					chartreport = chartreport+" to "+current_date+"(the data we have begins at "+begin_date+" and ends at "+end_date+")"+"\n";
        				}
        				else {
        					chartreport = chartreport+" to "+current_date+"(the data we have begins at "+begin_date+")"+"\n";	
        				}      			
    				}
        			else;
        		}
        	}
        	ArrayList<String> datelist = temp.getallDates(TaskC2.transformdate(begin), TaskC2.transformdate(end));
        	CategoryAxis xAxis = (CategoryAxis) linechart.getXAxis();
            ObservableList<String> categories = FXCollections.observableArrayList(datelist);
            xAxis.setCategories(categories);
            xAxis.setAutoRanging(true);
        	String nodatacountry="";
        	if(available_countrylist.size()!=selectedcountry.size()) {
        		for(String all_country:selectedcountry) {
        			String country="";
        			for(String country_have_data:available_countrylist) {
        				if(all_country.equals(country_have_data)) {
        					country = country_have_data;
        				}
        			}
        			if(!country.equals(all_country)) nodatacountry += all_country+" ";        			
        		}
        	}
        	if(!nodatacountry.equals("")) {
        		chartreport += "There is no available data in these days: "+nodatacountry+"\n";
        	}
        	Textconsolechart.setText(chartreport);
        }
        catch(Exception e){
        	Textconsolechart.setText("There is no data available in these days of all countries");
        }
	}
	
	/**
	 * it is used to return to the main ui (ui.fxml)
	 * @param event
	 * @throws IOException
	 * 
	 */
    @FXML
    void Returntoui(ActionEvent event) throws IOException {
    	Parent root = FXMLLoader.load(getClass().getResource("/ui.fxml"));
		Scene uiscene = new Scene(root);
		Stage uiwindow = (Stage) ((Node)event.getSource()).getScene().getWindow(); 
		uiwindow.setScene(uiscene);
		uiwindow.show();
    }
    
    /**
     * get the console in the chart ui
     * @return
     * 
     */
    public TextArea getTextconsolechart() {
    	return Textconsolechart;
    }
    
    /**
     * This class is used to create the node that can be hovered
     * @author HU, Wenbin
     *
     */
    class HoveredNode extends StackPane {
        HoveredNode(String country,String date,float value) {
          setPrefSize(6, 6);

          final Label label = createDataThresholdLabel(value);

          setOnMouseEntered(new EventHandler<MouseEvent>() {
            @Override public void handle(MouseEvent mouseEvent) {
              getChildren().setAll(label);
              Textconsolechart.setText("Country: "+country+", Date: "+date+ ", Confirmed(per Million): "+String.valueOf(value));
              setCursor(Cursor.NONE);
              toFront();
            }
          });
          setOnMouseExited(new EventHandler<MouseEvent>() {
            @Override public void handle(MouseEvent mouseEvent) {
              getChildren().clear();
              setCursor(Cursor.CROSSHAIR);
            }
          });
        }
        
        private Label createDataThresholdLabel(float value) {
          final Label label = new Label(value + "");
          label.getStyleClass().addAll("default-color0", "chart-line-symbol", "chart-series-line");
          label.setStyle("-fx-font-size: 12; -fx-font-weight: bold;");
          label.setTextFill(Color.BLUEVIOLET);
          label.setMinSize(Label.USE_PREF_SIZE, Label.USE_PREF_SIZE);
          return label;
        }
      }
    

}
