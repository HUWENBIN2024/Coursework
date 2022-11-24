package comp3111.covid;
import javafx.beans.property.SimpleStringProperty;

/**
 * utility class to transform the data stream into the form that can be shown on the table.
 * @author fangxiao
 * 
 */
public class UtilC {   // this is class is about utilization 
	private SimpleStringProperty country;
	private SimpleStringProperty totalcase;
	private SimpleStringProperty totalpermillion;
	
	/**
	 * constructor
	 * @param c
	 * @param tc
	 * @param tp
	 * @throws Exception
	 * 
	 */
	public UtilC(SimpleStringProperty c,SimpleStringProperty tc, SimpleStringProperty tp) throws Exception {
		country = c;
		totalcase = tc;
		totalpermillion = tp;
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
		return totalcase.get();
	}
	
	/**
	 * get the total case per million 
	 * @return
	 *
	 */
	public String getTotalpermillion() {
		return totalpermillion.get();
	}
	
}
