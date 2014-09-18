#ifndef MEMS_WORKER_HPP_
#define MEMS_WORKER_HPP_

void MemsWorkerStart(void);
void MPU6050ISR(EXTDriver *extp, expchannel_t channel);

#endif /* MEMS_WORKER_HPP_ */
