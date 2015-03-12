#ifndef LINK_MGR_H_
#define LINK_MGR_H_

class LinkMgr {
public:
  LinkMgr(void);
  void start(void);
  void stop(void);
private:
  thread_t *worker = NULL;
};

#endif /* LINK_MGR_H_ */
