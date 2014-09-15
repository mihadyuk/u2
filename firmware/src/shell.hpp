#ifndef SHELL_HPP_
#define SHELL_HPP_

class Shell {
public:
  Shell(void){return;}
  void start(SerialDriver *sdp){this->sdp = sdp; ready = true;}
  void stop(void){
    if (true == ready){
      sdStop(this->sdp);
      ready = false;
    }
  }
private:
  bool ready = false;
  SerialDriver *sdp = nullptr;
};

#endif /* SHELL_HPP_ */
