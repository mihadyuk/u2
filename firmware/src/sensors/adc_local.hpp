#ifndef ADC_LOCAL_HPP_
#define ADC_LOCAL_HPP_

void ADCInitLocal(void);
adcsample_t ADCget6v(void);
adcsample_t ADCgetMainVoltage(void);
adcsample_t ADCgetCurrent(void);
adcsample_t ADCgetMPXVtemp(void);
adcsample_t ADCgetMPXV(void);

#endif /* ADC_LOCAL_HPP_ */

