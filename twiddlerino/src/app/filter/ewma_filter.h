/**
 * @file ewma_filter.h
 * @brief Exponential Weight Moving Average filter 
 *        https://en.wikipedia.org/wiki/Exponential_smoothing
 *        Inspiration from https://github.com/Reefwing-Software/Reefwing-Filter/tree/main/examples 
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 */

#ifndef EWMA_FILTER_H_
#define EWMA_FILTER_H_

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

#include <stdint.h>

/******************************************************************************/
/*                                 C L A S S                                  */
/******************************************************************************/

//Basic (simple) exponential moving average filter
//{\displaystyle s_{t}=\alpha x_{t}+(1-\alpha )s_{t-1}=s_{t-1}+\alpha (x_{t}-s_{t-1}).}
class EwmaFilter {
  public:

    EwmaFilter(double alpha, double init_out) {
      this->alpha = alpha;
      this->last_output = init_out;
    }

    double operator()(double meas) {
      last_output = last_output + alpha*(meas-last_output);
      return last_output;
    }

  private:
    double alpha = 0.0;
    double last_output = 0.0;
};

#endif //EWMA_FILTER_H_