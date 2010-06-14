#ifndef SERIAL_READER_CALIBRATE_H
#define SERIAL_READER_CALIBRATE_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>

#include "serial_reader/serial_reader.h"
#include "serial_reader//common.h"

int initSerial(SerialReader* serialReader);

using namespace std;

class statData
{
  public:  

    int iterations;

    double  ave;
    double  sum;
    double  sumsq;
    double  variance;
    double* val;

    const char* name;
    const char* units;

    statData(int i, const char* name, const char* units)
    {
      this->name = name;
      this->units = units;
  
      iterations = i;

      val   = new double[iterations];
    }

    void update()
    {
      updateSum();
      updateAve();
      updateSumsq();
      updateVariance();
    }

    void updateSum()
    {
      sum = 0.0;

      for (int i = 0; i < iterations; i++)
        sum += val[i];
    }

    void updateAve()
    {
      ave = sum / (double)iterations;
    }

    void updateSumsq()
    {
      sumsq = 0.0;

      for (int i = 0; i < iterations; i++)
        sumsq += (ave - val[i])*(ave - val[i]);
    }
    void updateVariance()
    {
      variance = sumsq / (double)iterations;
    }

    void print()
    {
        cout << setw(15) << name           << "\t";
        cout << setw( 8) << units          << "\t";
        cout << setw(15) << ave            << "\t";
        cout << setw(15) << sqrt(variance) << "\t";
        cout << setw(15) << variance       << endl;
    }

    static void printHeader()
    {
      cout << "-------------------------------------------------------------------" << endl;
      cout << setw(15) << "NAME"     << "\t";
      cout << setw( 8) << "UNITS"    << "\t";
      cout << setw(15) << "AVERAGE"  << "\t";
      cout << setw(15) << "STD DEV"  << "\t";
      cout << setw(15) << "VARIANCE" << endl;
      cout << "-------------------------------------------------------------------" << endl;
    }
};

#endif

