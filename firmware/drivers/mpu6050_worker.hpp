#ifndef MPU6050_WORKER_HPP_
#define MPU6050_WORKER_HPP_


//class MPU6050Worker {
//public:
//  MPU6050Worker(void);
//private:
//  thread_t *worker = nullptr;
//};



void Mpu6050Init(void);
void Mpu6050ISR(EXTDriver *extp, expchannel_t channel);




#endif /* MPU6050_WORKER_HPP_ */
