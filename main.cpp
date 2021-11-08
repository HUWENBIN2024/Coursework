#include <iostream>
#include "Book.h"
#include "Toy.h"
#include "ProductVector.h"
using namespace std;

int main() {
    cout << "1. Make products and Test Product class" << endl;
    Book b1(1000.0, "Book 1", "Fiction", "ABC publisher", 1);

    Toy t1(200, "Unicorn", "CCC Manufacturer", false);
    Toy t2(800, "Spider", "CCC Manufacturer", true);

    b1.print();
    t1.print();
    t2.print();

    cout << "===================================== " << endl;
    cout << "===================================== " << endl;
    cout << endl;
    
    cout << "2. Make vectors and test" << endl;
    cout << "===================================== " << endl;
    cout << endl;
    
    cout << "2-1 Type1 int " << endl;
    ProductVector<int> int_vector;
    int_vector.push_back(3);
    int_vector.push_back(9);
    int_vector.push_back(2);
    int_vector.push_back(8);
    int_vector.push_back(6);
    int_vector.push_back(5);

    cout << "int_vector.get_value(1): " << int_vector.get_value(1) << endl;
    cout << "int_vector.get_value(2): " << int_vector.get_value(2) << endl;
    int_vector.swap(1,2);
    cout << "== After swap index 1 and 2 ==" << endl;
    cout << "int_vector.get_value(1): " << int_vector.get_value(1) << endl;
    cout << "int_vector.get_value(2): " << int_vector.get_value(2) << endl;

    cout << "size: " << int_vector.size() << endl;
    cout << "== After remove == " << endl;
    int_vector.remove(2);
    cout << "size: " << int_vector.size() << endl;
    cout << "int_vector.get_value(2): " << int_vector.get_value(2) << endl;

    cout << "===================================== " << endl;
    cout << endl;
    cout << "2-2 Type2 Book " << endl;
    ProductVector<Book> book_vector;

    book_vector.push_back(b1);
    book_vector.push_back(Book(200.0, "Book 2", "Fiction", "ABC publisher", 0.9));
    book_vector.push_back(Book(700.0, "Book 3", "Fiction", "ABC publisher", 0.3));
    book_vector.push_back(Book(400.0, "Book 4", "Fiction", "ABC publisher", 0.7));
    book_vector.push_back(Book(500.0, "Book 5", "Fiction", "ABC publisher", 0.5));
    
    cout << "book_vector.get_value(1): " << book_vector.get_value(1).get_product_name() << endl;
    cout << "book_vector.get_value(2): " << book_vector.get_value(2).get_product_name() << endl;
    book_vector.swap(1,2);
    cout << "== After swap index 1 and 2 == " << endl;
    cout << "book_vector.get_value(1): " << book_vector.get_value(1).get_product_name() << endl;
    cout << "book_vector.get_value(2): " << book_vector.get_value(2).get_product_name() << endl;

    cout << "size: " << book_vector.size() << endl;
    cout << "== After remove == " << endl;
    book_vector.remove(2);
    cout << "size: " << book_vector.size() << endl;
    cout << "book_vector.get_value(2): " << book_vector.get_value(2).get_product_name() << endl;

    cout << "===================================== " << endl;
    cout << endl;
    cout << "2-3 Type3 Toy " << endl;
    ProductVector<Toy> toy_vector;
    toy_vector.push_back(t1);
    toy_vector.push_back(t2);
    toy_vector.push_back(Toy(100, "Snake", "CCC Manufacturer", true));
    toy_vector.push_back(Toy(500, "Lion", "CCC Manufacturer", false));
    
    cout << "toy_vector.get_value(1): " << toy_vector.get_value(1).get_product_name() << endl;
    cout << "toy_vector.get_value(2): " << toy_vector.get_value(2).get_product_name() << endl;
    book_vector.swap(1,2);
    cout << "== After swap index 1 and 2 == " << endl;
    cout << "toy_vector.get_value(1): " << toy_vector.get_value(1).get_product_name() << endl;
    cout << "toy_vector.get_value(2): " << toy_vector.get_value(2).get_product_name() << endl;

    cout << "size: " << toy_vector.size() << endl;
    cout << "== After remove == " << endl;
    toy_vector.remove(2);
    cout << "size: " << toy_vector.size() << endl;
    cout << "toy_vector.get_value(2): " << toy_vector.get_value(2).get_product_name() << endl;

    cout << "===================================== " << endl;
    cout << "===================================== " << endl;
    cout << endl;
    cout << "3. Compare price " << endl;
    compare_price(int_vector.get_value(0), int_vector.get_value(1));
    compare_price<Product>(book_vector.get_value(0), book_vector.get_value(1));
    compare_price<Product>(toy_vector.get_value(0), toy_vector.get_value(1));
    
    compare_price<Product>(book_vector.get_value(0), toy_vector.get_value(1));
    compare_price<Product>(toy_vector.get_value(0), book_vector.get_value(1));
    
    compare_price<int, 300>(500);
    compare_price<Product, 300>(book_vector.get_value(0));
    compare_price<Product, 800>(toy_vector.get_value(0));
    
    return 0;
}