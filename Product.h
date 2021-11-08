#ifndef PRODUCT_H
#define PRODUCT_H
#include <iostream>
using namespace std;

class Product
{
    private: 
        double regular_price;
        string product_name;
    public: 
        Product() : regular_price(0), product_name("") {}
        Product(double regular_price, const string& product_name)
          : regular_price(regular_price), product_name(product_name) {}
        string get_product_name() const { return product_name; }
        double get_regular_price() const { return regular_price; }

    /* ===================================================================
        TODO: Complete the get_retail_price function, and print function.
        HINT: This function is a pure virtual function.
       =================================================================== */
       virtual double get_retail_price() const = 0;
       virtual void print() const = 0;
};

#endif
