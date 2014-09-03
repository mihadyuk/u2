
/*
Caution added by Martin L. Buchanan, mlb@backgroundtask.com, Wed 11/16/2005:

If number is the maximum unsigned int value, call it MAX_VAL, then the first
evaluation of NEXT(n, number), with n == 1, produces an overflow when
1 + MAX_VAL/1 is evaluated. For an unsigned type the overflow typically
wraps around and yields zero as the macro result and zero as the
overall function result.
*/
//#define NEXT(n, i)  (((n) + (i)/(n)) >> 1)
//
//uint32_t sqrti(uint32_t number){
//  uint32_t n  = 1;
//  uint32_t n1 = NEXT(n, number);
//
//  while(abs(n1 - n) > 1){
//    n  = n1;
//    n1 = NEXT(n, number);
//  }
//  while(n1*n1 > number)
//    n1--;
//  return n1;
//}


/* But the following algorithm is many times faster, even when you have
 * hardware multiplication and division. */
uint32_t sqrti(uint32_t x){
  uint32_t op, res, one;

  op = x;
  res = 0;

  /* "one" starts at the highest power of four <= than the argument. */
  one = 1 << 30;  /* second-to-top bit set */
  while (one > op) one >>= 2;

  while (one != 0) {
    if (op >= res + one) {
      op -= res + one;
      res += one << 1;  // <-- faster than 2 * one
    }
    res >>= 1; /* div by 2 */
    one >>= 2; /* div by 4 */
  }
  return res;
}
