#include "bookingsystem.h"

#include <iostream>

using namespace std;

int BookingSystem::nextID = 0;

/* ===================================================
    TODO: Complete the type conversion constructor.
          Here we assume to allocate same capacity for tennisball, tenniscourt and person.
          Try to add only one extra line of code!
    HINT: Reuse the general constuctor we define below.
   =================================================== */
BookingSystem::BookingSystem(int capacity)
{
    // Finish constructing with printing.
    new (this) BookingSystem(capacity, capacity, capacity);
    cout << "By using the conversion constructor!" << endl;
}

/* ===================================================
    TODO: Complete the other constructor
          numTennisBall, numTennisCourt, numPerson should be initialized to be 0
          maxNumTennisBall, maxNumTennisCourt, maxNumPerson should be initialized to their assigned values
   =================================================== */
BookingSystem::BookingSystem(int nTennisBall, int nTennisCourt, int capacity)
{
    SN = nextID++;
    // TODO: Allocate memory for tennisballs, tenniscourts and persons here.
    numTennisBall = 0;
    numTennisCourt = 0;
    numPerson = 0;
    maxNumPerson = capacity;
    maxNumTennisBall = nTennisBall;
    maxNumTennisCourt = nTennisCourt;
    tennisballs = new TennisBall *[maxNumTennisBall];
    tenniscourts = new TennisCourt *[maxNumTennisCourt];
    persons = new Person *[maxNumPerson];
    // Finish constructing with printing.
    cout << "#" << SN << " BookingSystem Constructed!" << endl;
}

/* ===================================================
    TODO: Complete the copy constructor
          Deep copy all data members of another restaurant
   =================================================== */
BookingSystem::BookingSystem(const BookingSystem &other)
{
    SN = nextID++;

    numTennisBall = other.numTennisBall;
    numTennisCourt = other.numTennisCourt;
    numPerson = other.numPerson;
    maxNumPerson = other.maxNumPerson;
    maxNumTennisBall = other.maxNumTennisBall;
    maxNumTennisCourt = other.maxNumTennisCourt;

    // TODO: Clone TennisBalls.
    this->tennisballs = new TennisBall *[maxNumTennisBall];
    for (int i = 0; i < other.numTennisBall; i++)
    {
        tennisballs[i] = new TennisBall(*other.tennisballs[i]);
    }

    // TODO: Clone TennisCourts.
    this->tenniscourts = new TennisCourt *[maxNumTennisCourt];
    for (int i = 0; i < other.numTennisCourt; i++)
    {
        tenniscourts[i] = new TennisCourt(*other.tenniscourts[i]);
    }

    // TODO: Clone Persons.
    this->persons = new Person *[maxNumPerson];
    for (int i = 0; i < other.numPerson; i++)
    {
        persons[i] = new Person(*other.persons[i]);
    }
    // Finish copying with printing.

    cout << "#" << SN << " BookingSystem Copied!" << endl;
}

/* ===================================================
    TODO: Complete the destructor
   =================================================== */
BookingSystem::~BookingSystem()
{
    // TODO: Destruct TennisBalls.
    for (int i = 0; i < numTennisBall; i++)
    {
        delete tennisballs[i];
        tennisballs[i] = nullptr;
    }
    delete[] tennisballs;
    tennisballs = nullptr;
    // TODO: Destruct TennisCourts.
    for (int i = 0; i < numTennisCourt; i++)
    {
        delete tenniscourts[i];
        tenniscourts[i] = nullptr;
    }
    delete[] tenniscourts;
    tenniscourts = nullptr;
    // TODO: Destruct Persons.
    for (int i = 0; i < numPerson; i++)
    {
        delete persons[i];
        persons[i] = nullptr;
    }
    delete[] persons;
    persons = nullptr;

    // Finish destructing with printing.

    cout << "#" << SN << " BookingSystem is Destructed." << endl;
}

void BookingSystem::releaseTennisBall()
{
    if (numTennisBall >= maxNumTennisBall)
    {
        cout << "Error: there is no TennisBall , release failed." << endl;
    }
    else
    {
        // TODO: a tennis ball is available by allocating a TennisBall object.
        tennisballs[numTennisBall] = new TennisBall;
        numTennisBall++;
    }
}

void BookingSystem::releaseTennisCourt()
{
    if (numTennisCourt >= maxNumTennisCourt)
    {
        cout << "Error: there is no TennisCourt , release failed." << endl;
    }
    else
    {
        // TODO: a tennis court is available by allocating a TennisCourt object.
        tenniscourts[numTennisCourt] = new TennisCourt;
        numTennisCourt++;
    }
}

void BookingSystem::makeBooking()
{
    if (numPerson >= maxNumPerson)
    {
        cout << "Error: Capacity Lack" << endl;
    }
    else if (numTennisBall < 1 || numTennisCourt < 1)
    {
        cout << "Error: no enough facilities for booking" << endl;
    }
    else
    {
        /* ===================================================
           TODO: accept person's booking by allocating a Person object. 
            The person would like to reserve a tennis basll and a tennis court.
            The ownership of the top tennisball and tenniscourt is then transferred to the Person object,
            which means the TennisBall object and the TennisCourt object are now pointed by the new Person object instead.
        =================================================== */
        persons[numPerson] = new Person(tennisballs[numTennisBall - 1], tenniscourts[numTennisCourt - 1]);
        numPerson++;
        numTennisBall--;
        numTennisCourt--;
        // Finish processing.
        cout << "Booking is successful!!!" << endl;
    }
}

void BookingSystem::log() const
{
    cout << "----------------------" << endl;
    cout << "| Booking System Log |" << endl;
    cout << "----------------------" << endl;
    cout << "#" << SN << " booking system has..." << endl;
    cout << "TennisBall: " << numTennisBall << "/" << maxNumTennisBall << endl;
    cout << "TennisCourt: " << numTennisCourt << "/" << maxNumTennisCourt << endl;
    cout << "Person: " << numPerson << "/" << maxNumPerson << endl;
}