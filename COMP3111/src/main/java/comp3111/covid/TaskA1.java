package comp3111.covid;

import java.util.LinkedList;

import org.apache.commons.csv.CSVRecord;

/**
 * This class is the main class for doing taskA1
 * @author fangxiao
 * 
 */
public class TaskA1{
	private LinkedList<String> datalist = new LinkedList<String>();
	private LinkedList<String> nocountrydata = new LinkedList<String>();
	
	/**
	 * This function is used to store the selected country to the data list
	 * @param specific_date
	 * @param selectedcountry
	 * @return
	 * 
	 */
	LinkedList<String> storedatacountry(String specific_date,LinkedList<String> selectedcountry) {   // this is to store data to generate table if it is a country
		
		
		for(int i=0;i<selectedcountry.size();i++) {
			String country = selectedcountry.get(i);
			String confirmedpermillion ="abcd";
			for(CSVRecord rec: DataAnalysis.getFileParser("COVID_Dataset_v1.0.csv")) {
				if(country.equals(rec.get("location"))) {
					if (rec.get("date").equals(specific_date)) {
						String confirmedCases = rec.get("total_cases");
						if (rec.get("total_cases").equals("")) {
							confirmedCases = "0";
						}
						confirmedpermillion = rec.get("total_cases_per_million");
						if(rec.get("total_cases_per_million").equals("")) {
							confirmedpermillion = "0";
						}
						String list = country +","+ confirmedCases +","+ confirmedpermillion;
						datalist.add(list);
						break;
					}
					else;
				}
				else;
			}
			if(confirmedpermillion.equals("abcd")) nocountrydata.add(country);	
		}
		return datalist;
	}
	
	/**
	 * return the country list which has no data
	 * @return
	 * 
	 */
	public LinkedList<String> getNocountrydata() {
		return nocountrydata;
	}
	
}