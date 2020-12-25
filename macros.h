#ifndef CIV_MACROS_H_
#define CIV_MACROS_H_

/* time supervision */
#define TIME_OVER(target,time) ((unsigned long)((time) - (target)) < 0x80000000U)




#endif // CIV_MACROS_H_