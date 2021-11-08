#include <iostream>
#include "Book.h"

using namespace std;

Book::Book() : Product(0, ""), genre(""), publisher(""), discount(0) {}

Book::Book(double regular_price, const string& product_name,
           const string& genre, const string& publisher,
           double discount) 
           : Product(regular_price, product_name), genre(genre),
             publisher(publisher), discount(discount) {}

string Book::get_genre() const{
    return genre;
}

string Book::get_publisher() const{
    return publisher;
}

double Book::get_discount() const {
    return discount;
}

double Book::get_retail_price() const {
    return get_regular_price()*get_discount();
}

void Book::print() const {
    cout << "Book: "<< endl;
    cout << "Product name: " << get_product_name() << ", Genre: " << get_genre()
         << ", Publisher: " << get_publisher() << ", Retail Price: " << get_retail_price() <<endl;
}
