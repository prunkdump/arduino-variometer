#ifndef VARIOTEST_H
#define VARIOTEST_H

#include <Arduino.h>
#include <LightFat16.h>
#include <vertaccel.h>

/********************/
/* The test class   */
/********************/

class VarioTest {
 public:
   void Test(int Number);

 private:
   void ErrorSound(void);
   void TestOK(void);
   void TestEND(void);
   bool TestSDCARD(void);
   bool TestSound(void);
   bool TestScreen(void);
   bool TestMS5611(void); 
};

extern Vertaccel vertaccel;

#endif
