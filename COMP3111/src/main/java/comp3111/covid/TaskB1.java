package comp3111.covid;

import java.util.LinkedList;

import org.apache.commons.csv.CSVRecord;

/**
 * This class is the main class for doing taskB1
 * @author HU, Wenbin
 * 
 */
public class TaskB1{
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
			String deathsPerMillion ="abcd";
		
			for(CSVRecord rec: DataAnalysis.getFileParser("COVID_Dataset_v1.0.csv")) {
				if(country.equals(rec.get("location"))) {
					if (rec.get("date").equals(specific_date)) {
						String deathsCases = rec.get("total_deaths");
						if (rec.get("total_deaths").equals("")) {
							deathsCases = "0";
						}
						deathsPerMillion = rec.get("total_deaths_per_million");
						if(rec.get("total_deaths_per_million").equals("")) {
							deathsPerMillion = "0";
						}
						
						String list = country +","+ deathsCases +","+ deathsPerMillion;
						datalist.add(list);
						break;
					}
					else;
				}
				else;
			}
			if(deathsPerMillion.equals("abcd")) nocountrydata.add(country);	
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