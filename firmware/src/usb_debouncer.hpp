#ifndef USB_DEBOUNCER_H_
#define USB_DEBOUNCER_H_

/**
 *
 */
class UsbDebouncer {
public:
  UsbDebouncer(void);
  bool update(void);
private:
  bool plugged_flag;
};

#endif /* USB_DEBOUNCER_H_ */
