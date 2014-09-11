#ifndef SHELL_HPP_
#define SHELL_HPP_

class Shell {
public:
  Shell(void){return;}
  void start(SerialDriver *sdp){this->sdp = sdp;}
  void stop(void){sdStop(this->sdp);}
private:
  SerialDriver *sdp = nullptr;
};

#endif /* SHELL_HPP_ */
