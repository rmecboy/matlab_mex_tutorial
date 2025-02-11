#include "user_config.h"

double num1 = 0;
double num2 = 0;
double num3 = 0;

uint8_t task_init(void)
{
    num1 = 0.1;
    num2 = 0.2;
    num3 = 0.3;
    
    return 0;
}

void task_main()
{
    if(num1 == 0.1)
    {
        num1 = num2 + num3;
    }
    else{
        num1 = num1 + 0.001f ;
    }
    mexPrintf("num1 = %d\n", num1);

}