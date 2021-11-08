#ifndef BOOK_H
#define BOOK_H

#include <string>
#include <iostream>
#include "Product.h"
using namespace std;

/* ==========================================================================
    TODO: Complete the class declaration of Book
    HINT: Remember to take the structure figure on the lab page as reference
          This class is an extended version of lab4 with modification.
   ========================================================================== */

class Book : public Product
{
private:
    string genre;
    string publisher;
    double discount;
public:
    Book();
    Book(double regular_price, const string& product_name, 
        const string& genre, const string& publisher, double discount);
    string get_genre() const;
    string get_publisher() const;
    double get_discount() const;
    double get_retail_price() const;
    void print() const;
};

#endif
