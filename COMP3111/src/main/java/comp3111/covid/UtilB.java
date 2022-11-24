package comp3111.covid;
import javafx.beans.property.SimpleStringProperty;

/**
 * utility class to transform the data stream into the form that can be shown on the table.
 * @author HU, Wenbin
 * 
 */
public class UtilB {   // this is class is about utilization 
	private SimpleStringProperty country;
	private SimpleStringProperty totalDeaths;
	private SimpleStringProperty totalDeathsPerMillion;
	
	/**
	 * constructor
	 * @param c
	 * @param tc
	 * @param tp
	 * @throws Exception
	 * 
	 */
	public UtilB(SimpleStringProperty c,SimpleStringProperty tc, SimpleStringProperty tp) throws Exception {
		country = c;
		totalDeaths = tc;
		totalDeathsPerMillion = tp;
	}
	
	/**
	 * get the country
	 * @return
	 *
	 */
	public String getCountry() {
		return country.get();
	}
	
	/**
	 * get the total case 
	 * @return
	 *
	 */
	public String getTotalcase() {
		return totalDeaths.get();
	}
	
	/**
	 * get the total case per million 
	 * @return
	 *
	 */
	public String getTotalpermillion() {
		return totalDeathsPerMillion.get();
	}
	
}
